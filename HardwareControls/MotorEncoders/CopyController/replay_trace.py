from pathlib import Path
import sys
import argparse
import json
import math
import time
from typing import Dict, List, Tuple

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
    run_wheel_speeds,
    wheel_counts_error,
    wheel_speed_from_count_error,
)
from Servos.clawBase import ClawBaseServo
from Servos.clawPincher import Servo270Positions
from Servos.rackpinion import Servo270 as RackPinionServo
from Servos.binDump import BinDumpServo
from Servos.falseFloor import Servo270 as FalseFloorServo

DEFAULT_CALIBRATION = MOTOR_ENCODERS_DIR / "robot_calibration.json"


class ReplayServoController:
    def __init__(self):
        self.claw_base = ClawBaseServo()
        self.claw_pincher = Servo270Positions(channel=0)
        self.rack_pinion = RackPinionServo(channel=5)
        self.bin_dump = BinDumpServo()
        self.false_floor = FalseFloorServo(channel=6)

        self.claw_base.retract()
        self.claw_pincher.center_closed()
        self.rack_pinion.lower()
        self.bin_dump.close()
        self.false_floor.close()

        self.claw_pincher_state = "closed"
        self.rack_pinion_up = False
        self.bin_dump_open = False
        self.false_floor_open = False
        self.claw_base_extended = False

    def snapshot(self) -> dict:
        return {
            "claw_base": "extended" if self.claw_base_extended else "retracted",
            "claw_pincher": self.claw_pincher_state,
            "rack_pinion": "raised" if self.rack_pinion_up else "lowered",
            "bin_dump": "open" if self.bin_dump_open else "closed",
            "false_floor": "open" if self.false_floor_open else "closed",
        }

    def apply_action(self, action: str) -> None:
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

    def apply_events(self, events: List[dict]) -> None:
        for event in events:
            action = event.get("action")
            if action:
                self.apply_action(action)

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


def load_trace(path: str) -> List[dict]:
    with open(path, "r", encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def normalize_motor_dict(d: dict, cast=float) -> Dict[int, float]:
    return {int(k): cast(v) for k, v in d.items()}


def interpolate_trace(records: List[dict], t: float) -> dict:
    if t <= records[0]["t"]:
        return {
            "t": records[0]["t"],
            "wheel_cmd_rev_s": normalize_motor_dict(records[0]["wheel_cmd_rev_s"], float),
            "encoder_counts": normalize_motor_dict(records[0]["encoder_counts"], int),
            "estimated_pose": records[0]["estimated_pose"],
            "marker": records[0].get("marker", False),
            "servo_state": records[0].get("servo_state", {}),
        }

    if t >= records[-1]["t"]:
        return {
            "t": records[-1]["t"],
            "wheel_cmd_rev_s": normalize_motor_dict(records[-1]["wheel_cmd_rev_s"], float),
            "encoder_counts": normalize_motor_dict(records[-1]["encoder_counts"], int),
            "estimated_pose": records[-1]["estimated_pose"],
            "marker": records[-1].get("marker", False),
            "servo_state": records[-1].get("servo_state", {}),
        }

    lo = 0
    hi = len(records) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if records[mid]["t"] < t:
            lo = mid
        else:
            hi = mid

    a = records[lo]
    b = records[hi]
    alpha = (t - a["t"]) / max(b["t"] - a["t"], 1e-9)

    def lerp(x, y):
        return x + alpha * (y - x)

    a_cmd = normalize_motor_dict(a["wheel_cmd_rev_s"], float)
    b_cmd = normalize_motor_dict(b["wheel_cmd_rev_s"], float)
    a_cnt = normalize_motor_dict(a["encoder_counts"], int)
    b_cnt = normalize_motor_dict(b["encoder_counts"], int)

    out = {
        "t": t,
        "wheel_cmd_rev_s": {},
        "encoder_counts": {},
        "estimated_pose": {},
        "marker": a.get("marker", False) or b.get("marker", False),
        "servo_state": a.get("servo_state", {}) or b.get("servo_state", {}),
    }
    for motor_id in MOTOR_ORDER:
        out["wheel_cmd_rev_s"][motor_id] = lerp(a_cmd.get(motor_id, 0.0), b_cmd.get(motor_id, 0.0))
        out["encoder_counts"][motor_id] = int(round(lerp(a_cnt.get(motor_id, 0), b_cnt.get(motor_id, 0))))
    for key in ("x_m", "y_m", "theta_rad"):
        out["estimated_pose"][key] = lerp(float(a["estimated_pose"][key]), float(b["estimated_pose"][key]))
    return out


def export_steps(records: List[dict], out_path: str, pos_thresh_m: float, theta_thresh_rad: float) -> None:
    steps = []
    first = records[0]
    steps.append({
        "t": first["t"],
        "target_pose": first["estimated_pose"],
        "target_counts": normalize_motor_dict(first["encoder_counts"], int),
        "wheel_cmd_rev_s": normalize_motor_dict(first["wheel_cmd_rev_s"], float),
        "servo_state": first.get("servo_state", {}),
        "servo_events": first.get("servo_events", []),
    })
    last_pose = first["estimated_pose"]

    for rec in records[1:]:
        pose = rec["estimated_pose"]
        dx = pose["x_m"] - last_pose["x_m"]
        dy = pose["y_m"] - last_pose["y_m"]
        dtheta = pose["theta_rad"] - last_pose["theta_rad"]
        if math.hypot(dx, dy) >= pos_thresh_m or abs(dtheta) >= theta_thresh_rad or rec.get("marker", False) or rec.get("servo_events"):
            steps.append({
                "t": rec["t"],
                "target_pose": pose,
                "target_counts": normalize_motor_dict(rec["encoder_counts"], int),
                "wheel_cmd_rev_s": normalize_motor_dict(rec["wheel_cmd_rev_s"], float),
                "servo_state": rec.get("servo_state", {}),
                "servo_events": rec.get("servo_events", []),
            })
            last_pose = pose

    Path(out_path).write_text(
        json.dumps({"steps": steps, "source": "teleop trace export"}, indent=2),
        encoding="utf-8",
    )


def apply_pending_servo_events(servos: ReplayServoController, records: List[dict], next_event_index: int, now: float) -> int:
    while next_event_index < len(records) and records[next_event_index]["t"] <= now:
        events = records[next_event_index].get("servo_events", [])
        if events:
            servos.apply_events(events)
            actions = [e.get("action") for e in events if e.get("action")]
            print(f"Replay servo actions @ {records[next_event_index]['t']:.2f}s: {actions}")
        next_event_index += 1
    return next_event_index


def replay_speed(controller: HiwonderMecanumController, servos: ReplayServoController, records: List[dict], rate_hz: float, log_path: str) -> None:
    period = 1.0 / rate_hz
    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}
    t0 = time.monotonic()
    next_event_index = 0

    with open(log_path, "w", encoding="utf-8") as f:
        while True:
            now = time.monotonic() - t0
            next_event_index = apply_pending_servo_events(servos, records, next_event_index, now)
            ref = interpolate_trace(records, now)
            run_wheel_speeds(controller, ref["wheel_cmd_rev_s"], label="Replay speed mode")

            actual_states = None
            for _ in range(3):
                try:
                    actual_states = controller.read_all_motors()
                    break
                except Exception as exc:
                    print(f"Replay read retry due to: {exc}")
                    time.sleep(0.01)

            if actual_states is None:
                print("Skipping this replay sample because encoder read failed")
                if now >= records[-1]["t"]:
                    break
                time.sleep(max(0.0, period))
                continue

            actual_counts = counts_dict_from_states(actual_states)
            err = wheel_counts_error(actual_counts, ref["encoder_counts"])
            dx, dy, dtheta = controller.estimate_robot_displacement(origin_counts, actual_counts)

            f.write(json.dumps({
                "t": now,
                "mode": "speed",
                "reference_counts": {str(k): int(v) for k, v in ref["encoder_counts"].items()},
                "actual_counts": {str(k): int(v) for k, v in actual_counts.items()},
                "count_error": {str(k): int(v) for k, v in err.items()},
                "reference_pose": ref["estimated_pose"],
                "actual_pose": {"x_m": dx, "y_m": dy, "theta_rad": dtheta},
                "servo_state": servos.snapshot(),
            }) + "\n")

            if now >= records[-1]["t"]:
                break
            time.sleep(max(0.0, period))

    controller.stop_all()


def replay_encoder_tracking(
    controller: HiwonderMecanumController,
    servos: ReplayServoController,
    records: List[dict],
    rate_hz: float,
    log_path: str,
    kp_counts: float,
    max_correction_rev_s: float,
    blend: float,
) -> None:
    period = 1.0 / rate_hz
    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}
    t0 = time.monotonic()
    prev_t = 0.0
    next_event_index = 0

    with open(log_path, "w", encoding="utf-8") as f:
        while True:
            now = time.monotonic() - t0
            dt = max(now - prev_t, 1e-3)
            prev_t = now
            next_event_index = apply_pending_servo_events(servos, records, next_event_index, now)
            ref = interpolate_trace(records, now)

            actual_states = None
            for _ in range(3):
                try:
                    actual_states = controller.read_all_motors()
                    break
                except Exception as exc:
                    print(f"Replay read retry due to: {exc}")
                    time.sleep(0.01)

            if actual_states is None:
                print("Skipping this replay sample because encoder read failed")
                if now >= records[-1]["t"]:
                    break
                time.sleep(max(0.0, period))
                continue

            actual_counts = counts_dict_from_states(actual_states)
            ref_counts = ref["encoder_counts"]
            err = wheel_counts_error(actual_counts, ref_counts)
            corrected = {}
            base_cmds = ref["wheel_cmd_rev_s"]

            for motor_id in MOTOR_ORDER:
                correction = wheel_speed_from_count_error(
                    controller=controller,
                    error_counts=err[motor_id],
                    motor_id=motor_id,
                    dt_s=dt,
                    kp=kp_counts,
                    max_correction_rev_s=max_correction_rev_s,
                )
                corrected[motor_id] = (1.0 - blend) * base_cmds[motor_id] + blend * (base_cmds[motor_id] + correction)

            run_wheel_speeds(controller, corrected, label="Replay encoder-tracking mode")
            dx, dy, dtheta = controller.estimate_robot_displacement(origin_counts, actual_counts)
            print(
                f"t={now:6.2f}s "
                f"err={[err[m] for m in MOTOR_ORDER]} "
                f"pose=({dx: .3f}, {dy: .3f}, {dtheta: .3f})"
            )

            f.write(json.dumps({
                "t": now,
                "mode": "encoder_tracking",
                "reference_counts": {str(k): int(v) for k, v in ref_counts.items()},
                "actual_counts": {str(k): int(v) for k, v in actual_counts.items()},
                "count_error": {str(k): int(v) for k, v in err.items()},
                "base_wheel_cmd_rev_s": {str(k): float(v) for k, v in base_cmds.items()},
                "corrected_wheel_cmd_rev_s": {str(k): float(v) for k, v in corrected.items()},
                "reference_pose": ref["estimated_pose"],
                "actual_pose": {"x_m": dx, "y_m": dy, "theta_rad": dtheta},
                "servo_state": servos.snapshot(),
            }) + "\n")

            if now >= records[-1]["t"]:
                break
            time.sleep(max(0.0, period))

    controller.stop_all()


def parse_args():
    parser = argparse.ArgumentParser(description="Replay trace using MotorController.py with servo replay")
    parser.add_argument("--trace", required=True)
    parser.add_argument("--mode", choices=["speed", "encoder"], default="encoder")
    parser.add_argument("--port", default=None, help="Override serial port from calibration JSON")
    parser.add_argument("--baud", type=int, default=None, help="Override baud rate from calibration JSON")
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--kp-counts", type=float, default=0.15)
    parser.add_argument("--max-correction-rev-s", type=float, default=0.20)
    parser.add_argument("--blend", type=float, default=1.0)
    parser.add_argument("--log", default="replay_log.jsonl")
    parser.add_argument("--export-steps", default="")
    parser.add_argument("--step-pos-thresh-in", type=float, default=1.0)
    parser.add_argument("--step-theta-thresh-deg", type=float, default=10.0)
    return parser.parse_args()


def main():
    args = parse_args()
    json_port, json_baud = load_serial_settings(args.calibration)
    port = args.port if args.port is not None else json_port
    baud = args.baud if args.baud is not None else json_baud

    records = load_trace(args.trace)
    if not records:
        raise RuntimeError("Trace file is empty")

    if args.export_steps:
        export_steps(
            records,
            args.export_steps,
            pos_thresh_m=args.step_pos_thresh_in * 0.0254,
            theta_thresh_rad=math.radians(args.step_theta_thresh_deg),
        )
        print(f"Exported routine steps to {args.export_steps}")

    controller = HiwonderMecanumController(
        port=port,
        baud=baud,
        calibration_file=args.calibration,
    )
    servos = ReplayServoController()
    controller.open()
    controller.stop_all()
    controller.reset_all_encoders()
    print(f"Using calibration: {args.calibration}")
    print(f"Using serial port: {port}")
    print(f"Using baud rate : {baud}")

    try:
        if args.mode == "speed":
            replay_speed(controller, servos, records, args.rate, args.log)
        else:
            replay_encoder_tracking(
                controller,
                servos,
                records,
                args.rate,
                args.log,
                args.kp_counts,
                args.max_correction_rev_s,
                args.blend,
            )
    finally:
        controller.stop_all()
        controller.close()
        servos.deinit()

    print(f"Saved replay log to {args.log}")


if __name__ == "__main__":
    main()
