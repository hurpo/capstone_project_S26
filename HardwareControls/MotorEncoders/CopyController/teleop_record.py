from pathlib import Path
import sys
import argparse
import json
import time

import pygame

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

DEFAULT_CALIBRATION = PARENT_DIR / "robot_calibration.json"


def load_serial_settings(calibration_path: str) -> tuple[str, int]:
    path = Path(calibration_path)
    data = json.loads(path.read_text(encoding="utf-8"))
    serial_cfg = data.get("serial", {})
    port = serial_cfg.get("port", "/dev/ttyACM0")
    baud = int(serial_cfg.get("baud", 1000000))
    return port, baud


def parse_args():
    parser = argparse.ArgumentParser(
        description="Xbox 360 teleop recorder using MotorController.py"
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
    """
    Only allow one of four directions from the left stick:
      - forward/reverse from Y
      - strafe left/right from X
    No diagonal left-stick movement is allowed.

    Strategy:
      - whichever axis has larger magnitude wins
      - the other axis is forced to zero
    """
    if abs(left_x) > abs(left_y):
        return left_x, 0.0
    elif abs(left_y) > abs(left_x):
        return 0.0, left_y
    else:
        # tie case: if both are zero, stay zero; otherwise prefer Y
        if left_x == 0.0 and left_y == 0.0:
            return 0.0, 0.0
        return 0.0, left_y


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
    controller.open()
    controller.stop_all()
    controller.reset_all_encoders()

    print("Teleop controls")
    print("  Left stick Y : forward/reverse")
    print("  Left stick X : strafe left/right")
    print("  Left stick   : 4-direction only, no diagonals")
    print("  Right stick X: rotate")
    print("  A button     : mark routine step")
    print("  B button     : save and exit")
    print("  X button     : reset encoder origin")
    print(f"Using calibration: {args.calibration}")
    print(f"Using serial port: {port}")
    print(f"Using baud rate : {baud}")

    markers = []
    trace_start = time.monotonic()
    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}

    with out_path.open("w", encoding="utf-8") as f:
        try:
            while True:
                loop_t0 = time.monotonic()
                pygame.event.pump()

                raw_left_x = apply_deadband(js.get_axis(0), args.deadband)
                raw_left_y = apply_deadband(js.get_axis(1), args.deadband)
                right_x = apply_deadband(js.get_axis(3), args.deadband)

                left_x, left_y = quantize_left_stick_4dir(raw_left_x, raw_left_y)

                a_pressed = js.get_button(0)
                b_pressed = js.get_button(1)
                x_pressed = js.get_button(2)

                if x_pressed:
                    controller.stop_all()
                    controller.reset_all_encoders()
                    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}
                    trace_start = time.monotonic()
                    print("Reset encoder origin")
                    time.sleep(0.25)
                    continue

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
                    sleep_time = period - (time.monotonic() - loop_t0)
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    continue

                counts = counts_dict_from_states(states)
                tps = tps_dict_from_states(states)
                rps = rps_dict_from_states(states)
                dx, dy, dtheta = controller.estimate_robot_displacement(origin_counts, counts)
                elapsed = time.monotonic() - trace_start

                record = {
                    "t": elapsed,
                    "joystick": {
                        "raw_left_x": raw_left_x,
                        "raw_left_y": raw_left_y,
                        "left_x": left_x,
                        "left_y": left_y,
                        "right_x": right_x,
                    },
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
                    "marker": bool(a_pressed),
                }
                f.write(json.dumps(record) + "\n")

                if a_pressed:
                    markers.append({
                        "t": elapsed,
                        "counts": {str(k): int(v) for k, v in counts.items()},
                        "pose": {"x_m": dx, "y_m": dy, "theta_rad": dtheta},
                    })
                    print(f"Marked step at {elapsed:.2f}s")

                print(
                    f"t={elapsed:6.2f}s  "
                    f"raw=({raw_left_x: .3f}, {raw_left_y: .3f})  "
                    f"quantized=({left_x: .3f}, {left_y: .3f})  "
                    f"cmd fwd={v_forward: .3f} m/s left={v_left: .3f} m/s rot={omega: .3f} rad/s  "
                    f"pose x={dx: .3f} y={dy: .3f} th={dtheta: .3f}"
                )

                if b_pressed:
                    break

                sleep_time = period - (time.monotonic() - loop_t0)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        finally:
            controller.stop_all()
            controller.close()
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