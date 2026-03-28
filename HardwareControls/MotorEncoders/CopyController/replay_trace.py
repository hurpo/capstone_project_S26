from pathlib import Path
import sys
import argparse
import json
import math
import time
from typing import Dict, List, Tuple, Optional

SCRIPT_DIR = Path(__file__).resolve().parent
PARENT_DIR = SCRIPT_DIR.parent
PROJECT_DIR = PARENT_DIR.parent.parent

for p in (SCRIPT_DIR, PARENT_DIR, PROJECT_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from MotorController import HiwonderMecanumController, MOTOR_ORDER
from motion_bridge import (
    counts_dict_from_states,
    run_wheel_speeds,
    wheel_counts_error,
    wheel_speed_from_count_error,
)
from StateControllers import State
from HardwareControls.Servos.clawBase import ClawBaseServo
from HardwareControls.Servos.clawPincher import Servo270Positions
from HardwareControls.Servos.rackpinion import Servo270 as RackPinionServo
from HardwareControls.Servos.binDump import BinDumpServo
from HardwareControls.Servos.falseFloor import Servo270 as FalseFloorServo

DEFAULT_CALIBRATION = SCRIPT_DIR.parent / "robot_calibration.json"


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

    def apply_action(self, action: str):
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

    def apply_events(self, events: List[dict]):
        for e in events:
            if "action" in e:
                self.apply_action(e["action"])

    def snapshot(self) -> dict:
        return {
            "claw_base": "extended" if self.claw_base_extended else "retracted",
            "claw_pincher": self.claw_pincher_state,
            "rack_pinion": "raised" if self.rack_pinion_up else "lowered",
            "bin_dump": "open" if self.bin_dump_open else "closed",
            "false_floor": "open" if self.false_floor_open else "closed",
        }

    def deinit(self) -> None:
        for servo in (self.claw_base, self.claw_pincher, self.rack_pinion, self.bin_dump, self.false_floor):
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


def normalize_motor_dict(d: dict, cast=float) -> Dict[int, float]:
    return {int(k): cast(v) for k, v in d.items()}


def interpolate_records(records: List[dict], t: float) -> dict:
    if t <= records[0]["t"]:
        return {
            "t": records[0]["t"],
            "wheel_cmd_rev_s": normalize_motor_dict(records[0]["wheel_cmd_rev_s"], float),
            "encoder_counts": normalize_motor_dict(records[0]["encoder_counts"], int),
            "estimated_pose": records[0]["estimated_pose"],
            "servo_state": records[0].get("servo_state", {}),
        }

    if t >= records[-1]["t"]:
        return {
            "t": records[-1]["t"],
            "wheel_cmd_rev_s": normalize_motor_dict(records[-1]["wheel_cmd_rev_s"], float),
            "encoder_counts": normalize_motor_dict(records[-1]["encoder_counts"], int),
            "estimated_pose": records[-1]["estimated_pose"],
            "servo_state": records[-1].get("servo_state", {}),
        }

    lo, hi = 0, len(records) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if records[mid]["t"] < t:
            lo = mid
        else:
            hi = mid

    a, b = records[lo], records[hi]
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
        "servo_state": a.get("servo_state", {}) or b.get("servo_state", {}),
    }
    for motor_id in MOTOR_ORDER:
        out["wheel_cmd_rev_s"][motor_id] = lerp(a_cmd.get(motor_id, 0.0), b_cmd.get(motor_id, 0.0))
        out["encoder_counts"][motor_id] = int(round(lerp(a_cnt.get(motor_id, 0), b_cnt.get(motor_id, 0))))
    for key in ("x_m", "y_m", "theta_rad"):
        out["estimated_pose"][key] = lerp(float(a["estimated_pose"][key]), float(b["estimated_pose"][key]))
    return out


def apply_pending_servo_events(servos: ReplayServoController, records: List[dict], next_event_index: int, now: float) -> int:
    while next_event_index < len(records) and records[next_event_index]["t"] <= now:
        events = records[next_event_index].get("servo_events", [])
        if events:
            servos.apply_events(events)
            actions = [e.get("action") for e in events if e.get("action")]
            print(f"Replay servo actions @ {records[next_event_index]['t']:.2f}s: {actions}")
        next_event_index += 1
    return next_event_index


def _execute_state_records(
    controller: HiwonderMecanumController,
    servos: ReplayServoController,
    records: List[dict],
    kp_counts: float,
    max_correction_rev_s: float,
    blend: float,
    rate_hz: float,
    log_fh=None,
    state_name: Optional[str] = None,
):
    period = 1.0 / rate_hz
    origin_counts = {motor_id: 0 for motor_id in MOTOR_ORDER}
    t0 = time.monotonic()
    prev_t = 0.0
    next_event_index = 0

    while True:
        now = time.monotonic() - t0
        dt = max(now - prev_t, 1e-3)
        prev_t = now

        next_event_index = apply_pending_servo_events(servos, records, next_event_index, now)
        ref = interpolate_records(records, now)

        actual_states = None
        for _ in range(3):
            try:
                actual_states = controller.read_all_motors()
                break
            except Exception as exc:
                print(f"Retry encoder read: {exc}")
                time.sleep(0.01)

        if actual_states is None:
            print("Skipping sample (encoder read failed)")
            if now >= records[-1]["t"]:
                break
            time.sleep(period)
            continue

        actual_counts = counts_dict_from_states(actual_states)
        ref_counts = ref["encoder_counts"]
        err = wheel_counts_error(actual_counts, ref_counts)

        base_cmds = ref["wheel_cmd_rev_s"]
        corrected = {}

        for motor_id in MOTOR_ORDER:
            correction = wheel_speed_from_count_error(
                controller=controller,
                error_counts=err[motor_id],
                motor_id=motor_id,
                dt_s=dt,
                kp=kp_counts,
                max_correction_rev_s=max_correction_rev_s,
            )
            corrected[motor_id] = (
                (1.0 - blend) * base_cmds[motor_id]
                + blend * (base_cmds[motor_id] + correction)
            )

        run_wheel_speeds(controller, corrected, label=f"Run state {state_name or ''}")

        dx, dy, dtheta = controller.estimate_robot_displacement(origin_counts, actual_counts)
        print(
            f"{state_name or 'STATE'} t={now:6.2f}s "
            f"err={[err[m] for m in MOTOR_ORDER]} "
            f"pose=({dx: .3f}, {dy: .3f}, {dtheta: .3f})"
        )

        if log_fh is not None:
            log_fh.write(json.dumps({
                "state": state_name,
                "t": now,
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
            controller.stop_all()
            break

        time.sleep(max(0.0, period))


def load_routine(routine_name: str) -> dict:
    routines_dir = SCRIPT_DIR / "routines"
    path = routines_dir / f"{routine_name}.json"
    if not path.exists():
        raise FileNotFoundError(f"Routine '{routine_name}' not found at {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def run_routine(
    routine_name: str,
    state: State,
    controller: HiwonderMecanumController = None,
    port: Optional[str] = None,
    baud: Optional[int] = None,
    calibration: str = str(DEFAULT_CALIBRATION),
    kp_counts: float = 0.15,
    max_correction_rev_s: float = 0.20,
    blend: float = 1.0,
    rate: float = 20.0,
    log_path: Optional[str] = None,
):
    routine = load_routine(routine_name)
    state_name = state.name
    records = routine.get("states", {}).get(state_name)

    if not records:
        print(f"[run_routine] No data recorded for state {state_name}, skipping.")
        return

    json_port, json_baud = load_serial_settings(calibration)
    port = port if port is not None else json_port
    baud = baud if baud is not None else json_baud

    owns_controller = controller is None
    if owns_controller:
        controller = HiwonderMecanumController(
            port=port,
            baud=baud,
            calibration_file=calibration,
        )
        controller.open()
        controller.stop_all()
        controller.reset_all_encoders()

    servos = ReplayServoController()
    log_fh = open(log_path, "a", encoding="utf-8") if log_path else None
    try:
        _execute_state_records(
            controller=controller,
            servos=servos,
            records=records,
            kp_counts=kp_counts,
            max_correction_rev_s=max_correction_rev_s,
            blend=blend,
            rate_hz=rate,
            log_fh=log_fh,
            state_name=state_name,
        )
    finally:
        if log_fh:
            log_fh.close()
        servos.deinit()
        if owns_controller:
            controller.stop_all()
            controller.close()


def parse_args():
    parser = argparse.ArgumentParser(description="Replay state-structured routine with encoder correction")
    parser.add_argument("--routine", required=True, help="Routine name without .json")
    parser.add_argument("--state", default="", help="Optional single state name; if omitted, replay all recorded states in order")
    parser.add_argument("--port", default=None, help="Override serial port from calibration JSON")
    parser.add_argument("--baud", type=int, default=None, help="Override baud from calibration JSON")
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--kp-counts", type=float, default=0.15)
    parser.add_argument("--max-correction-rev-s", type=float, default=0.20)
    parser.add_argument("--blend", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--log", default="state_replay_log.jsonl")
    return parser.parse_args()


def main():
    args = parse_args()
    routine = load_routine(args.routine)

    json_port, json_baud = load_serial_settings(args.calibration)
    port = args.port if args.port is not None else json_port
    baud = args.baud if args.baud is not None else json_baud

    controller = HiwonderMecanumController(
        port=port,
        baud=baud,
        calibration_file=args.calibration,
    )
    controller.open()
    controller.stop_all()
    controller.reset_all_encoders()

    try:
        if args.state:
            run_routine(
                routine_name=args.routine,
                state=State[args.state],
                controller=controller,
                calibration=args.calibration,
                kp_counts=args.kp_counts,
                max_correction_rev_s=args.max_correction_rev_s,
                blend=args.blend,
                rate=args.rate,
                log_path=args.log,
            )
        else:
            for state in State:
                if state in (State.IDLE, State.INIT, State.END):
                    continue
                if state.name in routine.get("states", {}):
                    print(f"\n=== Replaying state {state.name} ===")
                    run_routine(
                        routine_name=args.routine,
                        state=state,
                        controller=controller,
                        calibration=args.calibration,
                        kp_counts=args.kp_counts,
                        max_correction_rev_s=args.max_correction_rev_s,
                        blend=args.blend,
                        rate=args.rate,
                        log_path=args.log,
                    )
    finally:
        controller.stop_all()
        controller.close()


if __name__ == "__main__":
    main()
