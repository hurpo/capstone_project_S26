from pathlib import Path
import sys
import argparse
import json
import math
import time

SCRIPT_DIR = Path(__file__).resolve().parent
PARENT_DIR = SCRIPT_DIR.parent
if str(PARENT_DIR) not in sys.path:
    sys.path.insert(0, str(PARENT_DIR))

from MotorController import HiwonderMecanumController, MOTOR_ORDER
from motion_bridge import (
    counts_dict_from_states,
    run_wheel_speeds,
    wheel_counts_error,
    wheel_speed_from_count_error,
)

def normalize_motor_dict(d: dict, cast=float):
    return {int(k): cast(v) for k, v in d.items()}

def main():
    parser = argparse.ArgumentParser(description="Run exported routine steps using MotorController.py")
    parser.add_argument("--steps", required=True)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--calibration", default="robot_calibration.json")
    parser.add_argument("--kp-counts", type=float, default=0.15)
    parser.add_argument("--max-correction-rev-s", type=float, default=0.20)
    parser.add_argument("--tol-in", type=float, default=0.4)
    parser.add_argument("--tol-deg", type=float, default=6.0)
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()

    data = json.loads(open(args.steps, "r", encoding="utf-8").read())
    steps = data["steps"]

    controller = HiwonderMecanumController(
        port=args.port,
        baud=args.baud,
        calibration_file=args.calibration,
    )
    controller.open()
    controller.stop_all()
    controller.reset_all_encoders()

    pos_tol_m = args.tol_in * 0.0254
    theta_tol_rad = math.radians(args.tol_deg)
    period = 1.0 / args.rate
    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}

    try:
        for idx, step in enumerate(steps):
            ref_counts = normalize_motor_dict(step["target_counts"], int)
            base_cmds = normalize_motor_dict(step["wheel_cmd_rev_s"], float)
            tgt_pose = step["target_pose"]

            print(f"Executing step {idx+1}/{len(steps)}")
            while True:
                actual_states = controller.read_all_motors()
                actual_counts = counts_dict_from_states(actual_states)
                err = wheel_counts_error(actual_counts, ref_counts)

                corrected = {}
                for motor_id in MOTOR_ORDER:
                    correction = wheel_speed_from_count_error(
                        controller=controller,
                        error_counts=err[motor_id],
                        motor_id=motor_id,
                        dt_s=period,
                        kp=args.kp_counts,
                        max_correction_rev_s=args.max_correction_rev_s,
                    )
                    corrected[motor_id] = base_cmds.get(motor_id, 0.0) + correction

                run_wheel_speeds(controller, corrected, label="Run step")

                dx, dy, dtheta = controller.estimate_robot_displacement(origin_counts, actual_counts)
                pos_err = math.hypot(dx - tgt_pose["x_m"], dy - tgt_pose["y_m"])
                theta_err = abs(dtheta - tgt_pose["theta_rad"])

                print(
                    f"pose=({dx/0.0254:.2f} in, {dy/0.0254:.2f} in, {math.degrees(dtheta):.2f} deg) "
                    f"target=({tgt_pose['x_m']/0.0254:.2f} in, {tgt_pose['y_m']/0.0254:.2f} in, {math.degrees(tgt_pose['theta_rad']):.2f} deg) "
                    f"count_err={[err[m] for m in MOTOR_ORDER]}"
                )

                if pos_err <= pos_tol_m and theta_err <= theta_tol_rad:
                    controller.stop_all()
                    break

                time.sleep(period)
    finally:
        controller.stop_all()
        controller.close()

if __name__ == "__main__":
    main()
