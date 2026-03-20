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

DEFAULT_CALIBRATION = PARENT_DIR / "robot_calibration.json"


def load_serial_settings(calibration_path: str) -> tuple[str, int]:
    path = Path(calibration_path)
    data = json.loads(path.read_text(encoding="utf-8"))
    serial_cfg = data.get("serial", {})
    port = serial_cfg.get("port", "/dev/ttyACM0")
    baud = int(serial_cfg.get("baud", 1000000))
    return port, baud


def normalize_motor_dict(d: dict, cast=float):
    return {int(k): cast(v) for k, v in d.items()}


def parse_args():
    parser = argparse.ArgumentParser(description="Run exported routine steps using MotorController.py")
    parser.add_argument("--steps", required=True)
    parser.add_argument("--port", default=None, help="Override serial port from calibration JSON")
    parser.add_argument("--baud", type=int, default=None, help="Override baud rate from calibration JSON")
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--kp-counts", type=float, default=0.15)
    parser.add_argument("--max-correction-rev-s", type=float, default=0.20)
    parser.add_argument("--tol-in", type=float, default=0.4)
    parser.add_argument("--tol-deg", type=float, default=6.0)
    parser.add_argument("--rate", type=float, default=20.0)
    return parser.parse_args()


def main():
    args = parse_args()
    json_port, json_baud = load_serial_settings(args.calibration)
    port = args.port if args.port is not None else json_port
    baud = args.baud if args.baud is not None else json_baud

    data = json.loads(Path(args.steps).read_text(encoding="utf-8"))
    steps = data["steps"]

    controller = HiwonderMecanumController(
        port=port,
        baud=baud,
        calibration_file=args.calibration,
    )
    controller.open()
    controller.stop_all()
    controller.reset_all_encoders()
    print(f"Using calibration: {args.calibration}")
    print(f"Using serial port: {port}")
    print(f"Using baud rate : {baud}")

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
                actual_states = None
                for _ in range(3):
                    try:
                        actual_states = controller.read_all_motors()
                        break
                    except Exception as exc:
                        print(f"Step read retry due to: {exc}")
                        time.sleep(0.01)

                if actual_states is None:
                    print("Skipping this control cycle because encoder read failed")
                    time.sleep(period)
                    continue

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
