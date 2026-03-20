from pathlib import Path
import sys
import argparse
import json
import math
import time
from typing import Dict, List

SCRIPT_DIR = Path(__file__).resolve().parent
PARENT_DIR = SCRIPT_DIR.parent
PROJECT_DIR = PARENT_DIR.parent.parent

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


# ---------------------------------------------------------------------------
# Helpers — identical to replay_trace.py
# ---------------------------------------------------------------------------

def normalize_motor_dict(d: dict, cast=float) -> Dict[int, float]:
    return {int(k): cast(v) for k, v in d.items()}


def interpolate_records(records: List[dict], t: float) -> dict:
    if t <= records[0]["t"]:
        return {
            "t": records[0]["t"],
            "wheel_cmd_rev_s": normalize_motor_dict(records[0]["wheel_cmd_rev_s"], float),
            "encoder_counts":  normalize_motor_dict(records[0]["encoder_counts"], int),
            "estimated_pose":  records[0]["estimated_pose"],
        }

    if t >= records[-1]["t"]:
        return {
            "t": records[-1]["t"],
            "wheel_cmd_rev_s": normalize_motor_dict(records[-1]["wheel_cmd_rev_s"], float),
            "encoder_counts":  normalize_motor_dict(records[-1]["encoder_counts"], int),
            "estimated_pose":  records[-1]["estimated_pose"],
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
        "encoder_counts":  {},
        "estimated_pose":  {},
    }
    for motor_id in MOTOR_ORDER:
        out["wheel_cmd_rev_s"][motor_id] = lerp(a_cmd.get(motor_id, 0.0), b_cmd.get(motor_id, 0.0))
        out["encoder_counts"][motor_id]  = int(round(lerp(a_cnt.get(motor_id, 0), b_cnt.get(motor_id, 0))))
    for key in ("x_m", "y_m", "theta_rad"):
        out["estimated_pose"][key] = lerp(float(a["estimated_pose"][key]), float(b["estimated_pose"][key]))

    return out


# ---------------------------------------------------------------------------
# Core execution — mirrors replay_encoder_tracking from replay_trace.py
# ---------------------------------------------------------------------------

def _execute_state_records(
    controller: HiwonderMecanumController,
    records: List[dict],
    kp_counts: float,
    max_correction_rev_s: float,
    blend: float,
    rate_hz: float,
):
    period  = 1.0 / rate_hz
    t0      = time.monotonic()
    prev_t  = 0.0

    while True:
        now = time.monotonic() - t0
        dt  = max(now - prev_t, 1e-3)
        prev_t = now

        ref          = interpolate_records(records, now)
        actual_states = controller.read_all_motors()
        actual_counts = counts_dict_from_states(actual_states)

        # Offset ref counts to be relative to robot's current position.
        # Because encoder_counts in the JSON are saved relative to recording
        # origin (0-based), and actual_counts reflect whatever the hardware
        # currently reads, we anchor using the first record as the bridge.
        ref_counts = ref["encoder_counts"]
        err        = wheel_counts_error(actual_counts, ref_counts)

        base_cmds = ref["wheel_cmd_rev_s"]
        corrected  = {}
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
            print()  # newline after \r
            break

        time.sleep(max(0.0, period))


# ---------------------------------------------------------------------------
# Routine loading
# ---------------------------------------------------------------------------

def load_routine(routine_name: str) -> dict:
    routines_dir = SCRIPT_DIR / "routines"
    path = routines_dir / f"{routine_name}.json"
    if not path.exists():
        raise FileNotFoundError(f"Routine '{routine_name}' not found at {path}")
    return json.loads(path.read_text(encoding="utf-8"))


# ---------------------------------------------------------------------------
# Public API — call from state machine
# ---------------------------------------------------------------------------

def run_routine(
    routine_name: str,
    state: State,
    controller: HiwonderMecanumController = None,
    port: str = "/dev/ttyACM0",
    baud: int = 1000000,
    calibration: str = "robot_calibration.json",
    kp_counts: float = 0.15,
    max_correction_rev_s: float = 0.20,
    blend: float = 1.0,
    rate: float = 20.0,
):
    """
    Run the recorded data for a single state from a routine JSON.

    If controller is provided (e.g. called from a state machine) it will be
    reused and left open. Otherwise a new one is opened and closed automatically.
    """
    routine    = load_routine(routine_name)
    state_name = state.name
    records    = routine.get("states", {}).get(state_name)

    if not records:
        print(f"[run_routine] No data recorded for state {state_name}, skipping.")
        return

    print(f"[run_routine] Running state {state_name} ({len(records)} records, "
          f"duration={records[-1]['t']:.2f}s)")

    owns_controller = controller is None
    if owns_controller:
        controller = HiwonderMecanumController(
            port=port,
            baud=baud,
            calibration_file=calibration,
        )
        controller.open()
        controller.stop_all()

    try:
        _execute_state_records(
            controller=controller,
            records=records,
            kp_counts=kp_counts,
            max_correction_rev_s=max_correction_rev_s,
            blend=blend,
            rate_hz=rate,
        )
    finally:
        if owns_controller:
            controller.stop_all()
            controller.close()


# ---------------------------------------------------------------------------
# Direct execution — run all states in order
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Run a saved routine")
    parser.add_argument("--port",                  default="/dev/ttyACM0")
    parser.add_argument("--baud",                  type=int,   default=1000000)
    parser.add_argument("--calibration",           default="robot_calibration.json")
    parser.add_argument("--kp-counts",             type=float, default=0.15)
    parser.add_argument("--max-correction-rev-s",  type=float, default=0.20)
    parser.add_argument("--blend",                 type=float, default=1.0)
    parser.add_argument("--rate",                  type=float, default=20.0)
    args = parser.parse_args()

    # Pick routine interactively
    routines_dir = SCRIPT_DIR / "routines"
    available    = sorted(routines_dir.glob("*.json")) if routines_dir.exists() else []

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

    routine         = load_routine(routine_name)
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

    try:
        for state_name in recorded_states:
            records = routine["states"][state_name]
            if not records:
                print(f"\n=== State: {state_name} — no data, skipping. ===")
                continue
            print(f"\n=== State: {state_name} ({len(records)} records, "
                  f"duration={records[-1]['t']:.2f}s) ===")
            _execute_state_records(
                controller=controller,
                records=records,
                kp_counts=args.kp_counts,
                max_correction_rev_s=args.max_correction_rev_s,
                blend=args.blend,
                rate_hz=args.rate,
            )
    finally:
        controller.stop_all()
        controller.close()


if __name__ == "__main__":
    main()