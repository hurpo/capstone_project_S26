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

def parse_args():
    parser = argparse.ArgumentParser(description="Xbox 360 teleop recorder using MotorController.py")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--calibration", default="robot_calibration.json")
    parser.add_argument("--out", default="teleop_trace.jsonl")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-rev-s", type=float, default=0.6)
    parser.add_argument("--deadband", type=float, default=0.08)
    parser.add_argument("--joystick-index", type=int, default=0)
    return parser.parse_args()

def apply_deadband(x: float, d: float) -> float:
    return 0.0 if abs(x) < d else x

def main():
    args = parse_args()
    period = 1.0 / args.rate
    out_path = Path(args.out)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= args.joystick_index:
        raise RuntimeError("No joystick found at requested index")

    js = pygame.joystick.Joystick(args.joystick_index)
    js.init()

    controller = HiwonderMecanumController(
        port=args.port,
        baud=args.baud,
        calibration_file=args.calibration,
    )
    controller.open()
    controller.stop_all()
    controller.reset_all_encoders()

    print("Teleop controls")
    print("  Left stick Y : forward/reverse")
    print("  Left stick X : strafe")
    print("  Right stick X: rotate")
    print("  A button     : mark routine step")
    print("  B button     : save and exit")
    print("  X button     : reset encoder origin")

    markers = []
    trace_start = time.monotonic()
    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}

    with out_path.open("w", encoding="utf-8") as f:
        try:
            while True:
                loop_t0 = time.monotonic()
                pygame.event.pump()

                left_x = apply_deadband(js.get_axis(0), args.deadband)
                left_y = apply_deadband(js.get_axis(1), args.deadband)
                right_x = apply_deadband(js.get_axis(3), args.deadband)

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
                states = controller.read_all_motors()
                counts = counts_dict_from_states(states)
                tps = tps_dict_from_states(states)
                rps = rps_dict_from_states(states)
                dx, dy, dtheta = controller.estimate_robot_displacement(origin_counts, counts)
                elapsed = time.monotonic() - trace_start

                record = {
                    "t": elapsed,
                    "joystick": {"left_x": left_x, "left_y": left_y, "right_x": right_x},
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
    summary_path.write_text(json.dumps({"markers": markers, "source_trace": str(out_path)}, indent=2), encoding="utf-8")
    print(f"Saved trace to {out_path}")
    print(f"Saved marker summary to {summary_path}")

if __name__ == "__main__":
    main()
