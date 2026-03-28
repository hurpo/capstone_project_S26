from pathlib import Path
import sys
import argparse
import json
import time
import threading
import select
from typing import Dict, List, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
MOTOR_ENCODERS_DIR = SCRIPT_DIR.parent
HARDWARE_CONTROLS_DIR = MOTOR_ENCODERS_DIR.parent
PROJECT_DIR = HARDWARE_CONTROLS_DIR.parent

for p in (MOTOR_ENCODERS_DIR, HARDWARE_CONTROLS_DIR, PROJECT_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import pygame

from MotorController import HiwonderMecanumController, MOTOR_ORDER
from motion_bridge import (
    counts_dict_from_states,
    joystick_to_chassis_and_wheels,
    rps_dict_from_states,
    run_wheel_speeds,
    tps_dict_from_states,
)
from StateControllers import State

from Servos.clawBase import ClawBaseServo
from Servos.clawPincher import Servo270Positions
from Servos.rackpinion import Servo270 as RackPinionServo
from Servos.binDump import BinDumpServo
from Servos.falseFloor import Servo270 as FalseFloorServo

DEFAULT_CALIBRATION = MOTOR_ENCODERS_DIR / "robot_calibration.json"
ROUTINES_DIR = SCRIPT_DIR / "routines"
RECORDABLE_STATES = [s for s in State if s not in (State.IDLE, State.INIT, State.END)]

_remote_input: dict = {
    "btn_x": False, "btn_y": False, "btn_b": False, "btn_a": False,
    "btn_tl": False, "btn_tr": False, "btn_thumbl": False, "btn_thumbr": False,
    "btn_start": False, "btn_back": False,
    "left_x": 0.0, "left_y": 0.0, "right_x": 0.0, "right_y": 0.0,
    "trigger_l": 0.0, "trigger_r": 0.0, "d_x": 0.0, "d_y": 0.0,
}
_remote_input_lock = threading.Lock()
_signals: set = set()
_signals_lock = threading.Lock()
_send_popup_fn = None


def set_popup_callback(fn):
    global _send_popup_fn
    _send_popup_fn = fn


def _push_signal(sig: str):
    with _signals_lock:
        _signals.add(sig)


def _pop_signal(sig: str) -> bool:
    with _signals_lock:
        if sig in _signals:
            _signals.discard(sig)
            return True
    return False


def _kb_available() -> bool:
    try:
        return bool(select.select([sys.stdin], [], [], 0)[0])
    except Exception:
        return False


def _kb_read() -> str:
    if _kb_available():
        return sys.stdin.readline().strip().lower()
    return ""


def _poll_keyboard(direct: bool):
    if not direct:
        return
    key = _kb_read()
    if key == "r":
        _push_signal("record")
    elif key == "n":
        _push_signal("next")
    elif key == "b":
        _push_signal("back")
    elif key == "o":
        _push_signal("overwrite")
    elif key == "a":
        _push_signal("append")
    elif key == "y":
        _push_signal("ok")
    elif key == "d":
        _push_signal("disable_warn")
    elif key == "x":
        _push_signal("finish")


def _ensure_routines_dir():
    ROUTINES_DIR.mkdir(parents=True, exist_ok=True)


def _next_routine_name() -> str:
    _ensure_routines_dir()
    existing = list(ROUTINES_DIR.glob("routine_*.json"))
    indices = []
    for p in existing:
        parts = p.stem.split("_")
        if len(parts) == 2 and parts[1].isdigit():
            indices.append(int(parts[1]))
    n = max(indices) + 1 if indices else 0
    return f"routine_{n:03d}"


def _load_existing_routine(name: str) -> dict:
    path = ROUTINES_DIR / f"{name}.json"
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {}


def _save_routine(name: str, data: dict):
    _ensure_routines_dir()
    path = ROUTINES_DIR / f"{name}.json"
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"Saved routine to {path}")


def _wait_for_signal(direct: bool, *sigs: str, prompt: str = "") -> str:
    if prompt:
        print(prompt)
    while True:
        _poll_keyboard(direct)
        for sig in sigs:
            if _pop_signal(sig):
                return sig
        time.sleep(0.05)


def _prompt_overwrite_or_append(direct: bool, state: State) -> str:
    if _send_popup_fn:
        _send_popup_fn("OVERWRITE_OR_APPEND")
    return _wait_for_signal(
        direct, "overwrite", "append",
        prompt=f"State {state.name} already has data. [o]verwrite or [a]ppend?"
    )


def _prompt_position_warning(direct: bool, state: State) -> bool:
    if _send_popup_fn:
        _send_popup_fn("POSITION_WARNING")
    print(f"\nWARNING: Moving to state {state.name}. Robot position may not match the end position of the previous state.")
    print("Press [y/ok] to continue, [d/disable_warn] to continue and disable warning.")
    sig = _wait_for_signal(direct, "ok", "disable_warn")
    return sig == "disable_warn"


def _prompt_title(direct: bool) -> str:
    if _send_popup_fn:
        _send_popup_fn("PROMPT_TITLE")
    if direct:
        default = _next_routine_name()
        print(f"\nEnter routine name (leave blank for '{default}'): ", end="", flush=True)
        line = sys.stdin.readline().strip()
        return line if line else default
    while True:
        with _signals_lock:
            for sig in list(_signals):
                if sig.startswith("title:"):
                    _signals.discard(sig)
                    name = sig[len("title:"):].strip()
                    return name if name else _next_routine_name()
        time.sleep(0.05)


class TeleopServoController:
    def __init__(self):
        self.claw_base = ClawBaseServo()
        self.claw_pincher = Servo270Positions(channel=0)
        self.rack_pinion = RackPinionServo(channel=5)
        self.bin_dump = BinDumpServo()
        self.false_floor = FalseFloorServo(channel=6)

        self.claw_pincher_state = "closed"
        self.rack_pinion_up = False
        self.bin_dump_open = False
        self.false_floor_open = False
        self.claw_base_extended = False

        self.claw_base.retract()
        self.claw_pincher.center_closed()
        self.rack_pinion.lower()
        self.bin_dump.close()
        self.false_floor.close()

    def snapshot(self) -> dict:
        return {
            "claw_base": "extended" if self.claw_base_extended else "retracted",
            "claw_pincher": self.claw_pincher_state,
            "rack_pinion": "raised" if self.rack_pinion_up else "lowered",
            "bin_dump": "open" if self.bin_dump_open else "closed",
            "false_floor": "open" if self.false_floor_open else "closed",
        }

    def handle_actions(self, actions: List[str]) -> List[dict]:
        events: List[dict] = []
        for action in actions:
            if action == "claw_base_extend":
                self.claw_base.extend()
                self.claw_base_extended = True
            elif action == "claw_base_retract":
                self.claw_base.retract()
                self.claw_base_extended = False
            elif action == "claw_pincher_cycle":
                if self.claw_pincher_state == "closed":
                    self.claw_pincher.open()
                    self.claw_pincher_state = "open"
                elif self.claw_pincher_state == "open":
                    self.claw_pincher.latched()
                    self.claw_pincher_state = "latched"
                else:
                    self.claw_pincher.center_closed()
                    self.claw_pincher_state = "closed"
            elif action == "rack_pinion_toggle":
                if self.rack_pinion_up:
                    self.rack_pinion.lower()
                    self.rack_pinion_up = False
                else:
                    self.rack_pinion.raise_up()
                    self.rack_pinion_up = True
            elif action == "bin_dump_toggle":
                if self.bin_dump_open:
                    self.bin_dump.close()
                    self.bin_dump_open = False
                else:
                    self.bin_dump.open()
                    self.bin_dump_open = True
            elif action == "false_floor_open":
                self.false_floor.open()
                self.false_floor_open = True
            elif action == "false_floor_close":
                self.false_floor.close()
                self.false_floor_open = False
            else:
                continue
            events.append({"action": action, "state": self.snapshot()})
        return events

    def deinit(self) -> None:
        for servo in (
            self.claw_base,
            self.claw_pincher,
            self.rack_pinion,
            self.bin_dump,
            self.false_floor,
        ):
            try:
                servo.deinit()
            except Exception:
                pass


def load_serial_settings(calibration_path: str) -> Tuple[str, int]:
    path = Path(calibration_path)
    data = json.loads(path.read_text(encoding="utf-8"))
    serial_cfg = data.get("serial", {})
    port = serial_cfg.get("port", "/dev/ttyACM0")
    baud = int(serial_cfg.get("baud", 1000000))
    return port, baud


def parse_args():
    parser = argparse.ArgumentParser(description="State teleop recorder with encoder-rich records")
    parser.add_argument("--port", default=None, help="Override serial port from calibration JSON")
    parser.add_argument("--baud", type=int, default=None, help="Override baud rate from calibration JSON")
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-rev-s", type=float, default=0.6)
    parser.add_argument("--deadband", type=float, default=0.08)
    parser.add_argument("--joystick-index", type=int, default=0)
    parser.add_argument("--client", action="store_true", help="Receive controller input from socket client instead of local gamepad")
    parser.add_argument("--client-port", type=int, default=12346, help="Port to listen on for client controller data")
    return parser.parse_args()


def apply_deadband(x: float, d: float) -> float:
    return 0.0 if abs(x) < d else x


def quantize_left_stick_4dir(left_x: float, left_y: float) -> tuple[float, float]:
    if abs(left_x) > abs(left_y):
        return left_x, 0.0
    elif abs(left_y) > abs(left_x):
        return 0.0, left_y
    else:
        if left_x == 0.0 and left_y == 0.0:
            return 0.0, 0.0
        return 0.0, left_y


def get_hat(js: pygame.joystick.Joystick, hat_index: int = 0) -> Tuple[int, int]:
    if js.get_numhats() > hat_index:
        return js.get_hat(hat_index)
    return 0, 0


def rising_edge(current: bool, previous: bool) -> bool:
    return current and not previous


import socket as _socket_mod
import struct as _struct_mod

TYPE_JOYSTICK = b"J"
TYPE_SIGNAL = b"S"


def _recv_exact(conn, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed")
        buf += chunk
    return buf


def _client_listener(port: int):
    with _socket_mod.socket(_socket_mod.AF_INET, _socket_mod.SOCK_STREAM) as srv:
        srv.setsockopt(_socket_mod.SOL_SOCKET, _socket_mod.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", port))
        srv.listen(1)
        print(f"[Teleop] Waiting for client connection on port {port}...")
        conn, addr = srv.accept()
        print(f"[Teleop] Client connected from {addr}")
        with conn:
            while True:
                try:
                    mtype = _recv_exact(conn, 1)
                    (plen,) = _struct_mod.unpack("!I", _recv_exact(conn, 4))
                    payload = _recv_exact(conn, plen)
                    if mtype == TYPE_JOYSTICK:
                        data = json.loads(payload.decode("utf-8"))
                        with _remote_input_lock:
                            _remote_input.update(data)
                    elif mtype == TYPE_SIGNAL:
                        sig = payload.decode("utf-8").strip().lower()
                        _push_signal(sig)
                except Exception:
                    print("[Teleop] Client disconnected.")
                    break


def _read_input_direct(js, deadband: float):
    pygame.event.pump()
    raw_left_x = apply_deadband(js.get_axis(0), deadband)
    raw_left_y = apply_deadband(js.get_axis(1), deadband)
    right_x = apply_deadband(js.get_axis(3), deadband)
    left_x, left_y = quantize_left_stick_4dir(raw_left_x, raw_left_y)
    hat_x, hat_y = get_hat(js)
    current = {
        "a": bool(js.get_button(0)),
        "b": bool(js.get_button(1)),
        "x": bool(js.get_button(2)),
        "y": bool(js.get_button(3)),
        "lb": bool(js.get_button(4)),
        "back": bool(js.get_button(6)),
        "start": bool(js.get_button(7)),
        "dpad_up": hat_y > 0,
        "dpad_down": hat_y < 0,
    }
    return raw_left_x, raw_left_y, left_x, left_y, right_x, current


def _read_input_remote(deadband: float):
    with _remote_input_lock:
        raw_left_x = apply_deadband(float(_remote_input["left_x"]), deadband)
        raw_left_y = apply_deadband(float(_remote_input["left_y"]), deadband)
        right_x = apply_deadband(float(_remote_input["right_x"]), deadband)
        left_x, left_y = quantize_left_stick_4dir(raw_left_x, raw_left_y)
        current = {
            "a": bool(_remote_input["btn_a"]),
            "b": bool(_remote_input["btn_b"]),
            "x": bool(_remote_input["btn_x"]),
            "y": bool(_remote_input["btn_y"]),
            "lb": bool(_remote_input["btn_tl"]),
            "back": bool(_remote_input["btn_back"]),
            "start": bool(_remote_input["btn_start"]),
            "dpad_up": float(_remote_input["d_y"]) > 0.5,
            "dpad_down": float(_remote_input["d_y"]) < -0.5,
        }
    return raw_left_x, raw_left_y, left_x, left_y, right_x, current


def _record_state(controller, servos, state: State, args, direct: bool, js=None) -> List[dict]:
    period = 1.0 / args.rate
    records: List[dict] = []
    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}
    prev_buttons = {
        "a": False, "b": False, "x": False, "y": False,
        "lb": False, "back": False, "start": False,
        "dpad_up": False, "dpad_down": False,
    }

    print(f"\nRecording state {state.name}. Press record trigger again / signal to stop this state.")
    controller.reset_all_encoders()
    state_start = time.monotonic()

    while True:
        loop_t0 = time.monotonic()
        _poll_keyboard(direct)

        if direct:
            raw_left_x, raw_left_y, left_x, left_y, right_x, current = _read_input_direct(js, args.deadband)
        else:
            raw_left_x, raw_left_y, left_x, left_y, right_x, current = _read_input_remote(args.deadband)

        if rising_edge(current["start"], prev_buttons["start"]) or _pop_signal("record"):
            controller.stop_all()
            break

        if rising_edge(current["back"], prev_buttons["back"]):
            controller.stop_all()
            controller.reset_all_encoders()
            origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}
            state_start = time.monotonic()
            prev_buttons = current
            time.sleep(0.2)
            continue

        actions: List[str] = []
        if rising_edge(current["dpad_up"], prev_buttons["dpad_up"]):
            actions.append("claw_base_extend")
        if rising_edge(current["dpad_down"], prev_buttons["dpad_down"]):
            actions.append("claw_base_retract")
        if rising_edge(current["a"], prev_buttons["a"]):
            actions.append("claw_pincher_cycle")
        if rising_edge(current["x"], prev_buttons["x"]):
            actions.append("rack_pinion_toggle")
        if rising_edge(current["y"], prev_buttons["y"]):
            actions.append("bin_dump_toggle")
        if rising_edge(current["b"], prev_buttons["b"]):
            actions.append("false_floor_open")

        servo_events = servos.handle_actions(actions)
        v_forward, v_left, omega, wheel_cmds = joystick_to_chassis_and_wheels(
            controller=controller,
            left_y=left_y,
            left_x=left_x,
            right_x=right_x,
            max_rev_s=args.max_rev_s,
        )
        run_wheel_speeds(controller, wheel_cmds, label=f"Record {state.name}")

        states = None
        for _ in range(3):
            try:
                states = controller.read_all_motors()
                break
            except Exception as exc:
                print(f"Encoder read retry due to: {exc}")
                time.sleep(0.01)

        if states is None:
            prev_buttons = current
            sleep_time = period - (time.monotonic() - loop_t0)
            if sleep_time > 0:
                time.sleep(sleep_time)
            continue

        counts = counts_dict_from_states(states)
        tps = tps_dict_from_states(states)
        rps = rps_dict_from_states(states)
        dx, dy, dtheta = controller.estimate_robot_displacement(origin_counts, counts)
        elapsed = time.monotonic() - state_start
        marker_pressed = rising_edge(current["lb"], prev_buttons["lb"])

        record = {
            "t": elapsed,
            "state": state.name,
            "joystick": {
                "raw_left_x": raw_left_x,
                "raw_left_y": raw_left_y,
                "left_x": left_x,
                "left_y": left_y,
                "right_x": right_x,
            },
            "buttons": current,
            "chassis_cmd": {
                "v_forward_m_s": v_forward,
                "v_left_m_s": v_left,
                "omega_rad_s": omega,
            },
            "wheel_cmd_rev_s": {str(k): float(v) for k, v in wheel_cmds.items()},
            "encoder_counts": {str(k): int(v) for k, v in counts.items()},
            "encoder_tps": {str(k): float(v) for k, v in tps.items()},
            "encoder_rps": {str(k): float(v) for k, v in rps.items()},
            "estimated_pose": {"x_m": dx, "y_m": dy, "theta_rad": dtheta},
            "marker": bool(marker_pressed),
            "servo_events": servo_events,
            "servo_state": servos.snapshot(),
        }
        records.append(record)

        prev_buttons = current
        sleep_time = period - (time.monotonic() - loop_t0)
        if sleep_time > 0:
            time.sleep(sleep_time)

    controller.stop_all()
    return records


def _run(args):
    direct = not args.client
    if not direct:
        t = threading.Thread(target=_client_listener, args=(args.client_port,), daemon=True)
        t.start()

    js = None
    if direct:
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= args.joystick_index:
            raise RuntimeError("No joystick found")
        js = pygame.joystick.Joystick(args.joystick_index)
        js.init()

    json_port, json_baud = load_serial_settings(args.calibration)
    port = args.port if args.port is not None else json_port
    baud = args.baud if args.baud is not None else json_baud

    controller = HiwonderMecanumController(port=port, baud=baud, calibration_file=args.calibration)
    controller.open()
    controller.stop_all()
    controller.reset_all_encoders()
    servos = TeleopServoController()

    routine_name = _prompt_title(direct)
    routine = _load_existing_routine(routine_name) or {
        "routine_name": routine_name,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "states": {},
        "metadata": {
            "calibration": str(args.calibration),
            "rate_hz": args.rate,
            "max_rev_s": args.max_rev_s,
        },
    }

    print("\n=== State Teleop Recorder ===")
    print("States:", [s.name for s in RECORDABLE_STATES])

    disable_position_warning = False
    state_index = 0

    try:
        while 0 <= state_index < len(RECORDABLE_STATES):
            state = RECORDABLE_STATES[state_index]
            print(f"\nCurrent state: {state.name}")
            sig = _wait_for_signal(
                direct,
                "record", "next", "back", "finish",
                prompt="Choose: [r]ecord, [n]ext, [b]ack, [x]finish",
            )

            if sig == "finish":
                break
            if sig == "back":
                state_index = max(0, state_index - 1)
                continue
            if sig == "next":
                if not disable_position_warning and state_index > 0:
                    disable_position_warning = _prompt_position_warning(direct, state) or disable_position_warning
                state_index += 1
                continue

            existing = routine.get("states", {}).get(state.name)
            mode = _prompt_overwrite_or_append(direct, state) if existing else "overwrite"

            if not disable_position_warning and state_index > 0:
                disable_position_warning = _prompt_position_warning(direct, state) or disable_position_warning

            new_records = _record_state(controller, servos, state, args, direct, js=js)
            if mode == "append" and existing:
                routine["states"][state.name] = existing + new_records
            else:
                routine["states"][state.name] = new_records

            _save_routine(routine_name, routine)
            print(f"Recorded {len(new_records)} samples for state {state.name}")
            state_index += 1

    finally:
        controller.stop_all()
        controller.close()
        servos.deinit()
        if direct:
            pygame.quit()


def main():
    args = parse_args()
    _run(args)


if __name__ == "__main__":
    main()
