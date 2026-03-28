from pathlib import Path
import sys
import argparse
import json
import math
import time
from typing import Callable, Dict, List, Optional

SCRIPT_DIR = Path(__file__).resolve().parent
PARENT_DIR = SCRIPT_DIR.parent
PROJECT_DIR = PARENT_DIR.parent.parent

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PARENT_DIR) not in sys.path:
    sys.path.insert(0, str(PARENT_DIR))
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

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
        for event in events:
            if "action" in event:
                self.apply_action(event["action"])

    def snapshot(self) -> dict:
        return {
            "claw_base": "extended" if self.claw_base_extended else "retracted",
            "claw_pincher": self.claw_pincher_state,
            "rack_pinion": "raised" if self.rack_pinion_up else "lowered",
            "bin_dump": "open" if self.bin_dump_open else "closed",
            "false_floor": "open" if self.false_floor_open else "closed",
        }

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


def apply_pending_servo_events(servos: ReplayServoController, records: List[dict], next_event_index: int, now: float) -> int:
    while next_event_index < len(records) and records[next_event_index]["t"] <= now:
        events = records[next_event_index].get("servo_events", [])
        if events:
            servos.apply_events(events)
            actions = [e.get("action") for e in events if e.get("action")]
            print(f"Replay servo actions @ {records[next_event_index]['t']:.2f}s: {actions}")
        next_event_index += 1
    return next_event_index


def normalize_motor_dict(d: dict, cast=float) -> Dict[int, float]:
    return {int(k): cast(v) for k, v in d.items()}


def interpolate_records(records: List[dict], t: float) -> dict:
    if t <= records[0]["t"]:
        return {
            "t": records[0]["t"],
            "wheel_cmd_rev_s": normalize_motor_dict(records[0]["wheel_cmd_rev_s"], float),
            "encoder_counts": normalize_motor_dict(records[0]["encoder_counts"], int),
            "estimated_pose": records[0]["estimated_pose"],
        }

    if t >= records[-1]["t"]:
        return {
            "t": records[-1]["t"],
            "wheel_cmd_rev_s": normalize_motor_dict(records[-1]["wheel_cmd_rev_s"], float),
            "encoder_counts": normalize_motor_dict(records[-1]["encoder_counts"], int),
            "estimated_pose": records[-1]["estimated_pose"],
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
    }
    for motor_id in MOTOR_ORDER:
        out["wheel_cmd_rev_s"][motor_id] = lerp(a_cmd.get(motor_id, 0.0), b_cmd.get(motor_id, 0.0))
        out["encoder_counts"][motor_id] = int(round(lerp(a_cnt.get(motor_id, 0), b_cnt.get(motor_id, 0))))
    for key in ("x_m", "y_m", "theta_rad"):
        out["estimated_pose"][key] = lerp(float(a["estimated_pose"][key]), float(b["estimated_pose"][key]))

    return out


def _execute_state_records(
    controller: HiwonderMecanumController,
    servos: ReplayServoController,
    records: List[dict],
    kp_counts: float,
    max_correction_rev_s: float,
    blend: float,
    rate_hz: float,
):
    period = 1.0 / rate_hz
    t0 = time.monotonic()
    prev_t = 0.0
    next_event_index = 0

    while True:
        now = time.monotonic() - t0
        dt = max(now - prev_t, 1e-3)
        prev_t = now

        next_event_index = apply_pending_servo_events(servos, records, next_event_index, now)

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
                controller.stop_all()
                break
            time.sleep(max(0.0, period))
            continue

        ref = interpolate_records(records, now)
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

        run_wheel_speeds(controller, corrected, label="Run state")
        print(
            f"  t={now:6.2f}s  "
            f"err={[err[m] for m in MOTOR_ORDER]}  "
            f"pose=({ref['estimated_pose']['x_m']: .3f}, "
            f"{ref['estimated_pose']['y_m']: .3f}, "
            f"{ref['estimated_pose']['theta_rad']: .3f})",
            end="\r",
        )

        if now >= records[-1]["t"]:
            controller.stop_all()
            print()
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
    controller: Optional[HiwonderMecanumController] = None,
    port: str = "/dev/ttyACM0",
    baud: int = 1000000,
    calibration: str = SCRIPT_DIR.parent / "robot_calibration.json",
    kp_counts: float = 0.15,
    max_correction_rev_s: float = 0.20,
    blend: float = 1.0,
    rate: float = 20.0,
    pre_run_callback: Optional[Callable[[], None]] = None,
    post_run_callback: Optional[Callable[[], None]] = None,
):
    """
    Run the recorded data for a single state from a routine JSON.

    If controller is provided it will be reused and left open.
    Optional pre_run_callback / post_run_callback allow a caller (e.g. game.py)
    to tie other subsystems to the replay lifecycle.
    """
    routine = load_routine(routine_name)
    state_name = state.name
    records = routine.get("states", {}).get(state_name)

    if not records:
        print(f"[run_routine] No data recorded for state {state_name}, skipping.")
        if pre_run_callback is not None:
            pre_run_callback()
        if post_run_callback is not None:
            post_run_callback()
        return

    print(
        f"[run_routine] Running state {state_name} "
        f"({len(records)} records, duration={records[-1]['t']:.2f}s)"
    )

    owns_controller = controller is None
    if owns_controller:
        controller = HiwonderMecanumController(
            port=port,
            baud=baud,
            calibration_file=calibration,
        )
        controller.open()
        controller.stop_all()

    servos = ReplayServoController()
    try:
        if pre_run_callback is not None:
            pre_run_callback()

        controller.reset_all_encoders()
        _execute_state_records(
            controller=controller,
            servos=servos,
            records=records,
            kp_counts=kp_counts,
            max_correction_rev_s=max_correction_rev_s,
            blend=blend,
            rate_hz=rate,
        )

        if post_run_callback is not None:
            post_run_callback()
    finally:
        servos.deinit()
        if owns_controller:
            controller.stop_all()
            controller.close()


def main_from_name(
    routine_name: str = "main_routine",
    port: str = "/dev/ttyACM0",
    baud: int = 1000000,
    calibration: str = SCRIPT_DIR.parent / "robot_calibration.json",
    kp_counts: float = 0.05,
    max_correction_rev_s: float = 0.08,
    blend: float = 1.0,
    rate: float = 12.0,
):
    routine = load_routine(routine_name)
    recorded_states = list(routine.get("states", {}).keys())

    if not recorded_states:
        print(f"[run_states] No states found in routine '{routine_name}'.")
        return

    controller = HiwonderMecanumController(
        port=port,
        baud=baud,
        calibration_file=calibration,
    )
    controller.open()
    controller.stop_all()

    servos = ReplayServoController()
    try:
        for state_name in recorded_states:
            records = routine["states"][state_name]
            if not records:
                print(f"\n=== State: {state_name} — no data, skipping. ===")
                continue
            print(
                f"\n=== State: {state_name} "
                f"({len(records)} records, duration={records[-1]['t']:.2f}s) ==="
            )
            controller.reset_all_encoders()
            _execute_state_records(
                controller=controller,
                servos=servos,
                records=records,
                kp_counts=kp_counts,
                max_correction_rev_s=max_correction_rev_s,
                blend=blend,
                rate_hz=rate,
            )
    finally:
        controller.stop_all()
        controller.close()
        servos.deinit()


def main():
    parser = argparse.ArgumentParser(description="Run a saved routine")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--calibration", default=SCRIPT_DIR.parent / "robot_calibration.json")
    parser.add_argument("--kp-counts", type=float, default=0.15)
    parser.add_argument("--max-correction-rev-s", type=float, default=0.20)
    parser.add_argument("--blend", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()

    routines_dir = SCRIPT_DIR / "routines"
    available = sorted(routines_dir.glob("*.json")) if routines_dir.exists() else []

    if available:
        print("Available routines:")
        for i, p in enumerate(available):
            print(f"  [{i+1}] {p.stem}")
    else:
        print("No routines found in routines directory.")

    choice = input("\nEnter routine name (or leave blank for 'main_auto'): ").strip()

    if not choice:
        routine_name = "main_auto"
    elif choice.isdigit():
        idx = int(choice) - 1
        if 0 <= idx < len(available):
            routine_name = available[idx].stem
        else:
            print("Invalid selection, defaulting to 'main_auto'.")
            routine_name = "main_auto"
    else:
        routine_name = choice

    print(f"Running routine: {routine_name}\n")

    routine = load_routine(routine_name)
    recorded_states = list(routine.get("states", {}).keys())

    if not recorded_states:
        print("No states found in routine.")
        return

    controller = HiwonderMecanumController(
        port=args.port,
        baud=args.baud,
        calibration_file=args.calibration,
    )
    controller.open()
    controller.stop_all()

    servos = ReplayServoController()
    try:
        for state_name in recorded_states:
            records = routine["states"][state_name]
            if not records:
                print(f"\n=== State: {state_name} — no data, skipping. ===")
                continue
            print(
                f"\n=== State: {state_name} "
                f"({len(records)} records, duration={records[-1]['t']:.2f}s) ==="
            )
            _execute_state_records(
                controller=controller,
                servos=servos,
                records=records,
                kp_counts=args.kp_counts,
                max_correction_rev_s=args.max_correction_rev_s,
                blend=args.blend,
                rate_hz=args.rate,
            )
    finally:
        controller.stop_all()
        controller.close()
        servos.deinit()


if __name__ == "__main__":
    main()
