from pathlib import Path
import sys
import argparse
import json
import time
from typing import Dict, List, Tuple

import pygame

SCRIPT_DIR = Path(__file__).resolve().parent
MOTOR_ENCODERS_DIR = SCRIPT_DIR.parent
HARDWARE_CONTROLS_DIR = MOTOR_ENCODERS_DIR.parent
if str(MOTOR_ENCODERS_DIR) not in sys.path:
    sys.path.insert(0, str(MOTOR_ENCODERS_DIR))
if str(HARDWARE_CONTROLS_DIR) not in sys.path:
    sys.path.insert(0, str(HARDWARE_CONTROLS_DIR))

from MotorController import HiwonderMecanumController, MOTOR_ORDER
from motion_bridge import (
    counts_dict_from_states,
    joystick_to_chassis_and_wheels,
    rps_dict_from_states,
    run_wheel_speeds,
    tps_dict_from_states,
)
from Servos.clawBase import ClawBaseServo
from Servos.clawPincher import Servo270Positions
from Servos.rackpinion import Servo270 as RackPinionServo
from Servos.binDump import BinDumpServo
from Servos.falseFloor import Servo270 as FalseFloorServo

DEFAULT_CALIBRATION = MOTOR_ENCODERS_DIR / "robot_calibration.json"


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


def load_serial_settings(calibration_path: str) -> Tuple[str, int]:
    path = Path(calibration_path)
    data = json.loads(path.read_text(encoding="utf-8"))
    serial_cfg = data.get("serial", {})
    port = serial_cfg.get("port", "/dev/ttyACM0")
    baud = int(serial_cfg.get("baud", 1000000))
    return port, baud


def parse_args():
    parser = argparse.ArgumentParser(
        description="Xbox 360 teleop recorder using MotorController.py with servo trace recording"
    )
    parser.add_argument("--port", default=None, help="Override serial port from calibration JSON")
    parser.add_argument("--baud", type=int, default=None, help="Override baud rate from calibration JSON")
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--out", default="teleop_trace.jsonl")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-rev-s", type=float, default=0.6)
    parser.add_argument("--deadband", type=float, default=0.08)
    parser.add_argument("--joystick-index", type=int, default=0)
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


def main():
    args = parse_args()
    json_port, json_baud = load_serial_settings(args.calibration)
    port = args.port if args.port is not None else json_port
    baud = args.baud if args.baud is not None else json_baud

    period = 1.0 / args.rate
    out_path = Path(args.out)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= args.joystick_index:
        raise RuntimeError("No joystick found at requested index")

    js = pygame.joystick.Joystick(args.joystick_index)
    js.init()

    controller = HiwonderMecanumController(
        port=port,
        baud=baud,
        calibration_file=args.calibration,
    )
    servos = TeleopServoController()

    controller.open()
    controller.stop_all()
    controller.reset_all_encoders()

    print("Teleop controls")
    print("  Left stick Y : forward/reverse")
    print("  Left stick X : strafe left/right")
    print("  Left stick   : 4-direction only, no diagonals")
    print("  Right stick X: rotate")
    print("  D-pad up     : extend claw base")
    print("  D-pad down   : retract claw base")
    print("  A button     : claw pincher cycle closed -> open -> latched -> closed")
    print("  X button     : rack & pinion raise/lower toggle")
    print("  Y button     : bin dump open/close toggle")
    print("  B button     : false floor open")
    print("  Left bumper  : mark routine step")
    print("  Back         : reset encoder origin")
    print("  Start        : save and exit")
    print(f"Using calibration: {args.calibration}")
    print(f"Using serial port: {port}")
    print(f"Using baud rate : {baud}")

    markers = []
    trace_start = time.monotonic()
    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}
    prev_buttons: Dict[str, bool] = {
        "a": False,
        "b": False,
        "x": False,
        "y": False,
        "lb": False,
        "back": False,
        "start": False,
        "dpad_up": False,
        "dpad_down": False,
    }

    with out_path.open("w", encoding="utf-8") as f:
        try:
            while True:
                loop_t0 = time.monotonic()
                pygame.event.pump()

                raw_left_x = apply_deadband(js.get_axis(0), args.deadband)
                raw_left_y = apply_deadband(js.get_axis(1), args.deadband)
                right_x = apply_deadband(js.get_axis(3), args.deadband)
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

                if rising_edge(current["back"], prev_buttons["back"]):
                    controller.stop_all()
                    controller.reset_all_encoders()
                    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}
                    trace_start = time.monotonic()
                    print("Reset encoder origin")
                    prev_buttons = current
                    time.sleep(0.25)
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
                run_wheel_speeds(controller, wheel_cmds, label="Teleop wheel command")

                states = None
                for _ in range(3):
                    try:
                        states = controller.read_all_motors()
                        break
                    except Exception as exc:
                        print(f"Encoder read retry due to: {exc}")
                        time.sleep(0.01)

                if states is None:
                    print("Skipping this sample because encoder read failed")
                    prev_buttons = current
                    sleep_time = period - (time.monotonic() - loop_t0)
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    continue

                counts = counts_dict_from_states(states)
                tps = tps_dict_from_states(states)
                rps = rps_dict_from_states(states)
                dx, dy, dtheta = controller.estimate_robot_displacement(origin_counts, counts)
                elapsed = time.monotonic() - trace_start
                marker_pressed = rising_edge(current["lb"], prev_buttons["lb"])

                record = {
                    "t": elapsed,
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
                f.write(json.dumps(record) + "\n")

                if marker_pressed:
                    markers.append({
                        "t": elapsed,
                        "counts": {str(k): int(v) for k, v in counts.items()},
                        "pose": {"x_m": dx, "y_m": dy, "theta_rad": dtheta},
                        "servo_state": servos.snapshot(),
                    })
                    print(f"Marked step at {elapsed:.2f}s")

                if servo_events:
                    print(f"Servo actions @ {elapsed:.2f}s: {[e['action'] for e in servo_events]}")

                print(
                    f"t={elapsed:6.2f}s  "
                    f"raw=({raw_left_x: .3f}, {raw_left_y: .3f})  "
                    f"quantized=({left_x: .3f}, {left_y: .3f})  "
                    f"cmd fwd={v_forward: .3f} m/s left={v_left: .3f} m/s rot={omega: .3f} rad/s  "
                    f"pose x={dx: .3f} y={dy: .3f} th={dtheta: .3f}"
                )

                prev_buttons = current

                if rising_edge(current["start"], False):
                    # handled as current press within this loop only
                    break

                sleep_time = period - (time.monotonic() - loop_t0)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        finally:
            controller.stop_all()
            controller.close()
            servos.deinit()
            pygame.quit()

    summary_path = out_path.with_suffix(".summary.json")
    summary_path.write_text(
        json.dumps({"markers": markers, "source_trace": str(out_path)}, indent=2),
        encoding="utf-8",
    )
    print(f"Saved trace to {out_path}")
    print(f"Saved marker summary to {summary_path}")


if __name__ == "__main__":
    main()
