from pathlib import Path
import sys
import argparse
import json
import time
import threading
import select
from typing import List

SCRIPT_DIR = Path(__file__).resolve().parent
PARENT_DIR = SCRIPT_DIR.parent
if str(PARENT_DIR) not in sys.path:
    sys.path.insert(0, str(PARENT_DIR))

from MotorController import HiwonderMecanumController, MOTOR_ORDER
from motion_bridge import (
    counts_dict_from_states,
    joystick_to_chassis_and_wheels,
    rps_dict_from_states,
    run_wheel_speeds,
    tps_dict_from_states,
)

SCRIPT_DIR = Path(__file__).resolve().parent          # CopyController/
HARDWARE_DIR = SCRIPT_DIR.parent.parent               # HardwareControls/
PROJECT_DIR = HARDWARE_DIR.parent                     # capstone_project_S26/
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))
if str(HARDWARE_DIR) not in sys.path:
    sys.path.insert(0, str(HARDWARE_DIR))

from StateControllers import State

from HardwareControls.Servos.clawBase import ClawBaseServo
from HardwareControls.Servos.clawPincher import Servo270Positions
from HardwareControls.Servos.rackpinion import Servo270 as RackPinionServo
from HardwareControls.Servos.binDump import BinDumpServo
from HardwareControls.Servos.falseFloor import Servo270 as FalseFloorServo

# States to record (excluding INIT, IDLE, END)
RECORDABLE_STATES = [s for s in State if s not in (State.IDLE, State.INIT, State.END)]

ROUTINES_DIR = SCRIPT_DIR / "routines"

# ---------------------------------------------------------------------------
# Shared input state (used when receiving from client over socket)
# ---------------------------------------------------------------------------
_remote_input: dict = {
    "btn_x": False, "btn_y": False, "btn_b": False, "btn_a": False,
    "btn_tl": False, "btn_tr": False, "btn_thumbl": False, "btn_thumbr": False,
    "btn_start": False, "btn_back": False,
    "left_x": 0.0, "left_y": 0.0, "right_x": 0.0, "right_y": 0.0,
    "trigger_l": 0.0, "trigger_r": 0.0, "d_x": 0.0, "d_y": 0.0,
}
_remote_input_lock = threading.Lock()

_send_popup_fn = None



def set_popup_callback(fn):
    global _send_popup_fn
    _send_popup_fn = fn

# Signals sent from client
_signals: set = set()
_signals_lock = threading.Lock()

def _push_signal(sig: str):
    with _signals_lock:
        _signals.add(sig)

def _pop_signal(sig: str) -> bool:
    with _signals_lock:
        if sig in _signals:
            _signals.discard(sig)
            return True
    return False


# ---------------------------------------------------------------------------
# Keyboard helpers (direct mode)
# ---------------------------------------------------------------------------
def _kb_available() -> bool:
    """Non-blocking check for stdin input (Unix). On Windows always False."""
    try:
        return bool(select.select([sys.stdin], [], [], 0)[0])
    except Exception:
        return False

def _kb_read() -> str:
    if _kb_available():
        return sys.stdin.readline().strip().lower()
    return ""

def _poll_keyboard(direct: bool):
    """Map keyboard input to signals when running in direct mode."""
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


# ---------------------------------------------------------------------------
# Routines directory helpers
# ---------------------------------------------------------------------------
def _ensure_routines_dir():
    ROUTINES_DIR.mkdir(parents=True, exist_ok=True)

def _next_routine_name() -> str:
    _ensure_routines_dir()
    existing = list(ROUTINES_DIR.glob("routine_*.json"))
    indices = []
    for p in existing:
        stem = p.stem  # e.g. routine_003
        parts = stem.split("_")
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


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(description="Teleop recorder")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--calibration", default="../robot_calibration.json")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-rev-s", type=float, default=0.6)
    parser.add_argument("--deadband", type=float, default=0.08)
    parser.add_argument("--joystick-index", type=int, default=0)
    parser.add_argument("--client", action="store_true",
                        help="Receive controller input from socket client instead of local gamepad")
    parser.add_argument("--client-port", type=int, default=12346,
                        help="Port to listen on for client controller data")
    return parser.parse_args()

def apply_deadband(x: float, d: float) -> float:
    return 0.0 if abs(x) < d else x

def quantize_left_stick_4dir(left_x: float, left_y: float) -> tuple[float, float]:
    """
    Only allow one of four directions from the left stick:
      - forward/reverse from Y
      - strafe left/right from X
    No diagonal movement.

    Strategy:
      - whichever axis has larger magnitude wins
      - the other axis is forced to zero
    """
    if abs(left_x) > abs(left_y):
        return left_x, 0.0
    elif abs(left_y) > abs(left_x):
        return 0.0, left_y
    else:
        if left_x == 0.0 and left_y == 0.0:
            return 0.0, 0.0
        return 0.0, left_y

# ---------------------------------------------------------------------------
# Socket listener thread (client mode)
# ---------------------------------------------------------------------------
import socket as _socket_mod
import struct as _struct_mod

TYPE_JOYSTICK  = b"J"
TYPE_SIGNAL    = b"S"

def _client_listener(port: int):
    """Listens for joystick packets and signal packets from the client GUI."""
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

def _recv_exact(conn, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed")
        buf += chunk
    return buf


# ---------------------------------------------------------------------------
# Prompt helpers
# ---------------------------------------------------------------------------
def _wait_for_signal(direct: bool, *sigs: str, prompt: str = "") -> str:
    """Block until one of the given signals is received. Returns which one."""
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
        prompt=f"State {state.name} already has data. "
               f"[o]verwrite or [a]ppend?"
    )

def _prompt_position_warning(direct: bool, state: State) -> bool:
    if _send_popup_fn:
        _send_popup_fn("POSITION_WARNING")
    print(f"\n⚠ WARNING: Moving to state {state.name}. Robot position may not match "
          f"the end position of the previous state.")
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
    else:
        # Wait for a 'title:<name>' signal from client
        print("Waiting for routine title from client (send 'title:<name>' or 'title:' for default)...")
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

        # Ensure known startup positions.
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

def rising_edge(current: bool, previous: bool) -> bool:
    return current and not previous

def _run(args):
    direct = not args.client
    period = 1.0 / args.rate
    if not direct:
        print("🤬🤬🤬🤬🤬🤬🤬🤬🤬🤬🤬🤬: ", args)
        t = threading.Thread(target=_client_listener, args=(args.client_port,), daemon=True)
        t.start()

    # Set up local gamepad if direct
    js = None
    if direct:
        import pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= args.joystick_index:
            raise RuntimeError("No joystick found")
        js = pygame.joystick.Joystick(args.joystick_index)
        js.init()

    # Set up motor controller
    controller = HiwonderMecanumController(
        port=args.port,
        baud=args.baud,
        calibration_file=args.calibration,
    )
    controller.open()
    controller.stop_all()
    controller.reset_all_encoders()

    servos = TeleopServoController()

    print("\n=== Teleop Recorder ===")
    print("States:", [s.name for s in RECORDABLE_STATES])
    if direct:
        print("Keys: [r] record  [n] next state  [b] back  [o] overwrite  [a] append  [y] ok  [d] disable warn  [x] finish")
    else:
        print("Signals: record, next, back, overwrite, append, ok, disable_warn, finish, title:<name>")

    # Session state
    state_index = 0
    routine_data: dict = {"states": {}}   # {state_name: [records]}
    recording = False
    disable_position_warning = False
    first_move_in_state = True            # track if we need to show position warning
    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}
    trace_start = time.monotonic()

    prev_buttons = {
        "a": False,
        "b": False,
        "x": False,
        "y": False,
        "lb": False,
        "dpad_up": False,
        "dpad_down": False,
    }

    def current_state() -> State:
        return RECORDABLE_STATES[state_index]

    def get_input():
        if direct:
            import pygame
            pygame.event.pump()

            lx = apply_deadband(js.get_axis(0), args.deadband)
            ly = apply_deadband(js.get_axis(1), args.deadband)
            rx = apply_deadband(js.get_axis(3), args.deadband)
            lx, ly = quantize_left_stick_4dir(lx, ly)

            hat = js.get_hat(0) if js.get_numhats() > 0 else (0, 0)

            buttons = {
                "a": bool(js.get_button(0)),
                "b": bool(js.get_button(1)),
                "x": bool(js.get_button(2)),
                "y": bool(js.get_button(3)),
                "lb": bool(js.get_button(4)),
                "dpad_up": hat[1] > 0,
                "dpad_down": hat[1] < 0,
            }

        else:
            with _remote_input_lock:
                lx = apply_deadband(_remote_input["left_x"], args.deadband)
                ly = apply_deadband(_remote_input["left_y"], args.deadband)
                rx = apply_deadband(_remote_input["right_x"], args.deadband)

                buttons = {
                    "a": _remote_input["btn_a"],
                    "b": _remote_input["btn_b"],
                    "x": _remote_input["btn_x"],
                    "y": _remote_input["btn_y"],
                    "lb": _remote_input["btn_tl"],
                    "dpad_up": _remote_input["d_y"] > 0,
                    "dpad_down": _remote_input["d_y"] < 0,
                }

            lx, ly = quantize_left_stick_4dir(lx, ly)

        return lx, ly, rx, buttons
    
    print(f"\n[State 1/{len(RECORDABLE_STATES)}] {current_state().name} — press [r] to start recording")
    if _send_popup_fn:
        _send_popup_fn(f"STATE:{current_state().name}")

    try:
        while True:
            loop_t0 = time.monotonic()
            _poll_keyboard(direct)

            # --- Signal: next state ---
            if _pop_signal("next"):
                recording = False
                if state_index < len(RECORDABLE_STATES) - 1:
                    state_index += 1
                    first_move_in_state = True
                    print(f"\n[State {state_index+1}/{len(RECORDABLE_STATES)}] {current_state().name} — press [r] to start recording")
                    if _send_popup_fn:
                        _send_popup_fn(f"STATE:{current_state().name}")
                else:
                    print(f"\nYou are at the end of the routine ({current_state().name} is the last state).")
                    print("  [x] finish and save")
                    print("  [b] go back to a previous state")
                    print("  [n] return to current state and keep working")

            # --- Signal: back ---
            if _pop_signal("back"):
                recording = False
                if state_index > 0:
                    state_index -= 1
                    first_move_in_state = True
                    print(f"\n[State {state_index+1}/{len(RECORDABLE_STATES)}] {current_state().name} — press [r] to start recording")
                    if _send_popup_fn:
                        _send_popup_fn(f"STATE:{current_state().name}")
                else:
                    print("Already at first state.")

            # --- Signal: record ---
            if _pop_signal("record"):
                if not recording:
                    sname = current_state().name
                    existing = routine_data["states"].get(sname)
                    if existing:
                        choice = _prompt_overwrite_or_append(direct, current_state())
                        if choice == "overwrite":
                            routine_data["states"][sname] = []
                            print(f"Overwriting {sname}.")
                        else:
                            print(f"Appending to {sname}.")
                    else:
                        routine_data["states"][sname] = []

                    states_snapshot = controller.read_all_motors()
                    origin_counts = counts_dict_from_states(states_snapshot)

                    recording = True
                    first_move_in_state = False
                    trace_start = time.monotonic()
                    print(f"Recording {current_state().name}...")
                else:
                    recording = False
                    print(f"Stopped recording {current_state().name}.")

            # --- Signal: finish ---
            if _pop_signal("finish"):
                break


            # --- Get joystick ---
            left_x, left_y, right_x, current = get_input()
            actions = []

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
            marker_pressed = rising_edge(current["lb"], prev_buttons["lb"])

            servo_events = servos.handle_actions(actions)
            moving = any(abs(v) > 0.0 for v in (left_x, left_y, right_x))

            # --- Position warning on first move in new state ---
            if moving and first_move_in_state and not disable_position_warning and state_index > 0:
                controller.stop_all()
                disable_position_warning = _prompt_position_warning(direct, current_state())
                first_move_in_state = False
                continue
            elif moving:
                first_move_in_state = False

            # --- Drive motors ---
            v_forward, v_left, omega, wheel_cmds = joystick_to_chassis_and_wheels(
                controller=controller,
                left_y=left_y,
                left_x=left_x,
                right_x=right_x,
                max_rev_s=args.max_rev_s,
            )
            run_wheel_speeds(controller, wheel_cmds, label="Teleop")

            # --- Read encoders ---
            states = controller.read_all_motors()
            counts = counts_dict_from_states(states)
            tps = tps_dict_from_states(states)
            rps = rps_dict_from_states(states)
            dx, dy, dtheta = controller.estimate_robot_displacement(origin_counts, counts)
            elapsed = time.monotonic() - trace_start

            # --- Record if active ---
            if recording:
                sname = current_state().name
                record = {
                    "t": elapsed,
                    "joystick": {"left_x": left_x, "left_y": left_y, "right_x": right_x},
                    "chassis_cmd": {
                        "v_forward_m_s": v_forward,
                        "v_left_m_s": v_left,
                        "omega_rad_s": omega,
                    },
                    "wheel_cmd_rev_s": {str(k): float(v) for k, v in wheel_cmds.items()},
                    "encoder_counts": {str(k): int(v) - int(origin_counts[k]) for k, v in counts.items()},
                    "encoder_tps": {str(k): float(v) for k, v in tps.items()},
                    "encoder_rps": {str(k): float(v) for k, v in rps.items()},
                    "estimated_pose": {"x_m": dx, "y_m": dy, "theta_rad": dtheta},
                    "servo_events": servo_events,
                    "servo_state": servos.snapshot(),
                }
                routine_data["states"][sname].append(record)

            if servo_events:
                print(f"Servo actions @ {elapsed:.2f}s: {[e['action'] for e in servo_events]}")

            print(
                f"[{'REC' if recording else '   '}] {current_state().name:<20} "
                f"fwd={v_forward: .3f} left={v_left: .3f} rot={omega: .3f} "
                f"pose x={dx: .3f} y={dy: .3f} th={dtheta: .3f}",
                end="\r"
            )

            sleep_time = period - (time.monotonic() - loop_t0)
            if sleep_time > 0:
                time.sleep(sleep_time)
            prev_buttons = current
    finally:
        controller.stop_all()
        controller.close()
        servos.deinit()
        if direct and js:
            import pygame
            pygame.quit()

    # --- Save ---
    title = _prompt_title(direct)
    _save_routine(title, routine_data)
    print(f"\nDone. Routine saved as '{title}'.")

def start(port: str = "/dev/ttyACM0", baud: int = 1000000, rate: float = 20.0):
    import argparse
    args = argparse.Namespace(
        port=port,
        baud=baud,
        calibration=str(SCRIPT_DIR.parent / "robot_calibration.json"),
        rate=rate,
        max_rev_s=0.6,
        deadband=0.08,
        client=True,
        client_port=12346,
    )
    print("Running Auto-Builder from Client")
    _run(args)
    

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    args = parse_args()
    _run(args)


if __name__ == "__main__":
    main()