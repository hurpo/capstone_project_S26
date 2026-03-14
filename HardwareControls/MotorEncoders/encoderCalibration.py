"""
Interactive encoder calibration tool for the Hiwonder ROS Robot Control board.

What it does
- Walks the user through calibration for each wheel
- Supports multiple trials per motor
- Computes counts-per-wheel-revolution from manually rotated wheel tests
- Can optionally estimate counts-per-meter from straight-line drive tests
- Saves all raw trial data and averages to JSON and CSV logs

Assumptions
- Firmware supports these encoder commands on FUNC_MOTOR=0x03:
    0x10 read one motor
    0x11 read all motors
    0x12 reset one motor
    0x13 reset all motors
- Motor IDs are 0..3
- Motor run/stop commands use the same framing as the earlier host scripts

You should still verify:
- which motor ID maps to which wheel position
- which sign corresponds to forward motion on each wheel
"""
# run with python3 encoder_calibration_tool.py --port /dev/ttyACM0 --baud 1000000
from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import serial


HEADER = bytes([0xAA, 0x55])
FUNC_MOTOR = 0x03

CMD_MOTOR_RUN_SINGLE = 0x00
CMD_MOTOR_RUN_MULTI = 0x01
CMD_MOTOR_STOP_SINGLE = 0x02
CMD_MOTOR_STOP_MASK = 0x03

CMD_ENCODER_READ_ONE = 0x10
CMD_ENCODER_READ_ALL = 0x11
CMD_ENCODER_RESET_ONE = 0x12
CMD_ENCODER_RESET_ALL = 0x13

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1000000

WHEEL_DIAMETER_IN = 2.25
WHEEL_DIAMETER_M = WHEEL_DIAMETER_IN * 0.0254
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M

MOTOR_LABELS = {
    0: "front-left?",
    1: "rear-left?",
    2: "front-right?",
    3: "rear-right?",
}


@dataclass
class MotorState:
    motor_id: int
    count: int
    tps: float
    rps: float


@dataclass
class RotationTrial:
    motor_id: int
    motor_label: str
    trial_index: int
    manual_revolutions: float
    start_count: int
    end_count: int
    delta_count: int
    counts_per_rev: float
    timestamp: str


@dataclass
class DistanceTrial:
    distance_m: float
    start_counts: dict[int, int]
    end_counts: dict[int, int]
    delta_counts: dict[int, int]
    avg_abs_counts: float
    counts_per_meter: float
    timestamp: str


@dataclass
class CalibrationSession:
    port: str
    baud: int
    wheel_diameter_in: float
    wheel_diameter_m: float
    wheel_circumference_m: float
    created_at: str
    rotation_trials: list[RotationTrial] = field(default_factory=list)
    distance_trials: list[DistanceTrial] = field(default_factory=list)

    def rotation_summary(self) -> dict[int, dict]:
        grouped: dict[int, list[RotationTrial]] = {}
        for t in self.rotation_trials:
            grouped.setdefault(t.motor_id, []).append(t)

        out = {}
        for motor_id, trials in grouped.items():
            vals = [abs(t.counts_per_rev) for t in trials]
            avg = sum(vals) / len(vals)
            spread = max(vals) - min(vals) if len(vals) > 1 else 0.0
            out[motor_id] = {
                "motor_label": trials[0].motor_label,
                "num_trials": len(vals),
                "avg_counts_per_rev": avg,
                "min_counts_per_rev": min(vals),
                "max_counts_per_rev": max(vals),
                "spread": spread,
            }
        return out

    def distance_summary(self) -> dict:
        if not self.distance_trials:
            return {}
        vals = [t.counts_per_meter for t in self.distance_trials]
        avg = sum(vals) / len(vals)
        return {
            "num_trials": len(vals),
            "avg_counts_per_meter": avg,
            "min_counts_per_meter": min(vals),
            "max_counts_per_meter": max(vals),
            "spread": (max(vals) - min(vals)) if len(vals) > 1 else 0.0,
        }


def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ 0x8C) & 0xFF
            else:
                crc = (crc >> 1) & 0xFF
    return crc


def build_packet(function_code: int, payload: bytes) -> bytes:
    length = len(payload)
    body = bytes([function_code, length]) + payload
    checksum = crc8_maxim(body)
    return HEADER + body + bytes([checksum])


def hexdump(data: bytes) -> str:
    return data.hex(" ")


def send_packet(ser: serial.Serial, packet: bytes, label: str = "") -> None:
    if label:
        print(f"\n{label}")
    print("TX:", hexdump(packet))
    ser.write(packet)
    ser.flush()


def read_exact_packet(ser: serial.Serial, timeout_s: float = 1.0) -> Optional[bytes]:
    deadline = time.time() + timeout_s
    state = 0
    buf = bytearray()

    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue

        val = b[0]
        if state == 0:
            if val == 0xAA:
                buf.clear()
                buf.append(val)
                state = 1
            continue
        if state == 1:
            if val == 0x55:
                buf.append(val)
                state = 2
            else:
                buf.clear()
                state = 0
            continue
        if state == 2:
            buf.append(val)
            state = 3
            continue
        if state == 3:
            buf.append(val)
            payload_len = val
            tail = ser.read(payload_len + 1)
            if len(tail) != payload_len + 1:
                return None
            buf.extend(tail)
            return bytes(buf)
    return None


def validate_packet(packet: bytes) -> tuple[int, bytes]:
    if len(packet) < 5:
        raise ValueError("Packet too short")
    if packet[:2] != HEADER:
        raise ValueError("Bad header")
    function_code = packet[2]
    length = packet[3]
    payload = packet[4:4 + length]
    rx_crc = packet[4 + length]
    calc_crc = crc8_maxim(packet[2:4 + length])
    if calc_crc != rx_crc:
        raise ValueError(f"Bad CRC: rx=0x{rx_crc:02X} calc=0x{calc_crc:02X}")
    return function_code, payload


def transact(ser: serial.Serial, packet: bytes, timeout_s: float = 1.0, label: str = "") -> tuple[int, bytes]:
    ser.reset_input_buffer()
    send_packet(ser, packet, label=label)
    rx = read_exact_packet(ser, timeout_s=timeout_s)
    if rx is None:
        raise TimeoutError("Timed out waiting for response")
    print("RX:", hexdump(rx))
    return validate_packet(rx)


def motor_run_single(motor_id: int, speed_rps: float) -> bytes:
    payload = bytes([CMD_MOTOR_RUN_SINGLE, motor_id]) + struct.pack("<f", speed_rps)
    return build_packet(FUNC_MOTOR, payload)


def motor_stop_single(motor_id: int) -> bytes:
    payload = bytes([CMD_MOTOR_STOP_SINGLE, motor_id])
    return build_packet(FUNC_MOTOR, payload)


def motor_stop_mask(mask: int) -> bytes:
    payload = bytes([CMD_MOTOR_STOP_MASK, mask & 0xFF])
    return build_packet(FUNC_MOTOR, payload)


def encoder_read_one(motor_id: int) -> bytes:
    payload = bytes([CMD_ENCODER_READ_ONE, motor_id])
    return build_packet(FUNC_MOTOR, payload)


def encoder_read_all() -> bytes:
    payload = bytes([CMD_ENCODER_READ_ALL])
    return build_packet(FUNC_MOTOR, payload)


def encoder_reset_one(motor_id: int) -> bytes:
    payload = bytes([CMD_ENCODER_RESET_ONE, motor_id])
    return build_packet(FUNC_MOTOR, payload)


def encoder_reset_all() -> bytes:
    payload = bytes([CMD_ENCODER_RESET_ALL])
    return build_packet(FUNC_MOTOR, payload)


def parse_single_motor_payload(payload: bytes) -> MotorState:
    expected_len = 18
    if len(payload) != expected_len:
        raise ValueError(f"Single response length mismatch: {len(payload)} != {expected_len}")
    motor_id = payload[1]
    count = struct.unpack_from("<q", payload, 2)[0]
    tps = struct.unpack_from("<f", payload, 10)[0]
    rps = struct.unpack_from("<f", payload, 14)[0]
    return MotorState(motor_id=motor_id, count=count, tps=tps, rps=rps)


def parse_all_motor_payload(payload: bytes) -> list[MotorState]:
    if len(payload) < 2:
        raise ValueError("All response too short")
    motor_num = payload[1]
    offset = 2
    states = []
    for _ in range(motor_num):
        motor_id = payload[offset]
        count = struct.unpack_from("<q", payload, offset + 1)[0]
        tps = struct.unpack_from("<f", payload, offset + 9)[0]
        rps = struct.unpack_from("<f", payload, offset + 13)[0]
        states.append(MotorState(motor_id=motor_id, count=count, tps=tps, rps=rps))
        offset += 17
    return states


def stop_all(ser: serial.Serial) -> None:
    send_packet(ser, motor_stop_mask(0x0F), "Stop all motors")
    time.sleep(0.15)


def read_motor_state(ser: serial.Serial, motor_id: int) -> MotorState:
    func, payload = transact(ser, encoder_read_one(motor_id), label=f"Read motor {motor_id}")
    if func != FUNC_MOTOR:
        raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
    return parse_single_motor_payload(payload)


def read_all_motor_states(ser: serial.Serial) -> list[MotorState]:
    func, payload = transact(ser, encoder_read_all(), label="Read all motor states")
    if func != FUNC_MOTOR:
        raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
    return parse_all_motor_payload(payload)


def reset_motor(ser: serial.Serial, motor_id: int) -> MotorState:
    func, payload = transact(ser, encoder_reset_one(motor_id), label=f"Reset motor {motor_id}")
    if func != FUNC_MOTOR:
        raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
    return parse_single_motor_payload(payload)


def reset_all_motors(ser: serial.Serial) -> list[MotorState]:
    func, payload = transact(ser, encoder_reset_all(), label="Reset all motors")
    if func != FUNC_MOTOR:
        raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
    return parse_all_motor_payload(payload)


def prompt_float(prompt: str, min_value: Optional[float] = None) -> float:
    while True:
        raw = input(prompt).strip()
        try:
            val = float(raw)
        except ValueError:
            print("Enter a valid number.")
            continue
        if min_value is not None and val < min_value:
            print(f"Enter a value >= {min_value}.")
            continue
        return val


def prompt_int(prompt: str, valid: Optional[set[int]] = None, min_value: Optional[int] = None) -> int:
    while True:
        raw = input(prompt).strip()
        try:
            val = int(raw)
        except ValueError:
            print("Enter a valid integer.")
            continue
        if min_value is not None and val < min_value:
            print(f"Enter a value >= {min_value}.")
            continue
        if valid is not None and val not in valid:
            print(f"Enter one of: {sorted(valid)}")
            continue
        return val


def prompt_yes_no(prompt: str, default: bool = True) -> bool:
    suffix = "[Y/n]" if default else "[y/N]"
    raw = input(f"{prompt} {suffix} ").strip().lower()
    if not raw:
        return default
    return raw in ("y", "yes")


def print_rotation_summary(session: CalibrationSession) -> None:
    summary = session.rotation_summary()
    if not summary:
        print("\nNo rotation data collected yet.")
        return
    print("\nRotation summary")
    print("----------------")
    for motor_id in sorted(summary):
        s = summary[motor_id]
        print(
            f"motor {motor_id} ({s['motor_label']}): "
            f"avg={s['avg_counts_per_rev']:.3f} counts/rev over {s['num_trials']} trial(s), "
            f"range=[{s['min_counts_per_rev']:.3f}, {s['max_counts_per_rev']:.3f}]"
        )


def print_distance_summary(session: CalibrationSession) -> None:
    s = session.distance_summary()
    if not s:
        print("\nNo straight-distance data collected yet.")
        return
    print("\nStraight-line summary")
    print("---------------------")
    print(
        f"avg={s['avg_counts_per_meter']:.3f} counts/m over {s['num_trials']} trial(s), "
        f"range=[{s['min_counts_per_meter']:.3f}, {s['max_counts_per_meter']:.3f}]"
    )
    est = s["avg_counts_per_meter"] * WHEEL_CIRCUMFERENCE_M
    print(f"Implied counts/rev from straight-line data: {est:.3f}")


def do_manual_rotation_calibration(ser: serial.Serial, session: CalibrationSession) -> None:
    print("\nManual wheel rotation calibration")
    print("--------------------------------")
    print("This estimates counts per wheel revolution.")
    print("Recommended method:")
    print("  - lift the robot so the selected wheel can spin freely")
    print("  - choose a wheel/motor")
    print("  - reset its encoder")
    print("  - manually rotate that wheel an exact number of turns")
    print("  - read the final count")
    print("Use more turns (e.g. 5 or 10) for a better average.")

    while True:
        motor_id = prompt_int("Motor ID to calibrate (0-3): ", valid={0, 1, 2, 3})
        label = MOTOR_LABELS.get(motor_id, f"motor-{motor_id}")
        trials = prompt_int(f"How many trials for motor {motor_id} ({label})? ", min_value=1)

        for idx in range(1, trials + 1):
            print(f"\nTrial {idx}/{trials} for motor {motor_id} ({label})")
            reset_motor(ser, motor_id)
            input("Press Enter when ready to start manual rotation... ")
            start_state = read_motor_state(ser, motor_id)
            print(f"Starting count: {start_state.count}")

            revs = prompt_float("How many exact wheel revolutions will you rotate? ", min_value=0.001)
            input("Rotate the wheel now, then press Enter when done... ")

            end_state = read_motor_state(ser, motor_id)
            delta = end_state.count - start_state.count
            counts_per_rev = delta / revs

            trial = RotationTrial(
                motor_id=motor_id,
                motor_label=label,
                trial_index=idx,
                manual_revolutions=revs,
                start_count=start_state.count,
                end_count=end_state.count,
                delta_count=delta,
                counts_per_rev=counts_per_rev,
                timestamp=datetime.now().isoformat(timespec="seconds"),
            )
            session.rotation_trials.append(trial)

            print("\nRecorded trial")
            print("--------------")
            print(f"delta_count      = {delta}")
            print(f"manual_revs      = {revs}")
            print(f"counts_per_rev   = {counts_per_rev:.6f}")
            if delta == 0:
                print("Warning: no count change observed. Check the selected motor or encoder wiring.")

        print_rotation_summary(session)
        if not prompt_yes_no("Calibrate another motor manually?", default=False):
            break


def do_straight_distance_calibration(ser: serial.Serial, session: CalibrationSession) -> None:
    print("\nStraight-line drive calibration")
    print("-------------------------------")
    print("This estimates counts per meter based on real floor travel.")
    print("Use this after you already know the correct motor directions.")
    print("You will place the robot at a start mark, let it travel a measured distance,")
    print("then enter the actual measured distance traveled.")

    speed_rps = prompt_float("Commanded motor speed magnitude in rps (e.g. 0.3): ", min_value=0.001)
    run_time = prompt_float("Open-loop run time in seconds (e.g. 1.5): ", min_value=0.01)
    trials = prompt_int("How many trials? ", min_value=1)

    for idx in range(1, trials + 1):
        print(f"\nStraight-line trial {idx}/{trials}")
        reset_all_motors(ser)
        input("Place robot at start mark, then press Enter... ")

        # Assumes left side forward, right side reverse for user’s convention.
        send_packet(ser, motor_run_single(0, +speed_rps), label="Run motor 0")
        send_packet(ser, motor_run_single(1, +speed_rps), label="Run motor 1")
        send_packet(ser, motor_run_single(2, -speed_rps), label="Run motor 2")
        send_packet(ser, motor_run_single(3, -speed_rps), label="Run motor 3")
        time.sleep(run_time)
        stop_all(ser)

        states = read_all_motor_states(ser)
        end_counts = {s.motor_id: s.count for s in states}
        delta_counts = end_counts.copy()

        print("\nCounts after run:")
        for s in states:
            print(f"  motor {s.motor_id}: count={s.count}, tps={s.tps:.3f}, rps={s.rps:.3f}")

        distance_m = prompt_float("Enter the ACTUAL measured straight distance traveled in meters: ", min_value=0.0001)
        avg_abs_counts = sum(abs(v) for v in delta_counts.values()) / len(delta_counts)
        counts_per_meter = avg_abs_counts / distance_m

        trial = DistanceTrial(
            distance_m=distance_m,
            start_counts={0: 0, 1: 0, 2: 0, 3: 0},
            end_counts=end_counts,
            delta_counts=delta_counts,
            avg_abs_counts=avg_abs_counts,
            counts_per_meter=counts_per_meter,
            timestamp=datetime.now().isoformat(timespec="seconds"),
        )
        session.distance_trials.append(trial)

        print("\nRecorded straight-line trial")
        print("----------------------------")
        print(f"distance_m        = {distance_m:.6f}")
        print(f"avg_abs_counts    = {avg_abs_counts:.6f}")
        print(f"counts_per_meter  = {counts_per_meter:.6f}")

    print_distance_summary(session)
    print("\nNote:")
    print("counts_per_meter is often more accurate for floor travel than pure counts/rev,")
    print("because it includes wheel slip, floor compression, and real robot behavior.")


def show_live_counts(ser: serial.Serial) -> None:
    print("\nLive encoder monitor")
    print("--------------------")
    print("Press Ctrl+C to stop.")
    try:
        while True:
            states = read_all_motor_states(ser)
            line = " | ".join(
                f"m{s.motor_id}: count={s.count:>8d}, tps={s.tps:>8.3f}, rps={s.rps:>7.3f}"
                for s in states
            )
            print(line)
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nStopped live monitor.")


def save_logs(session: CalibrationSession, output_dir: Path) -> tuple[Path, Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = output_dir / f"encoder_calibration_{ts}.json"
    rot_csv_path = output_dir / f"encoder_rotation_trials_{ts}.csv"
    dist_csv_path = output_dir / f"encoder_distance_trials_{ts}.csv"

    payload = {
        "session": {
            "port": session.port,
            "baud": session.baud,
            "wheel_diameter_in": session.wheel_diameter_in,
            "wheel_diameter_m": session.wheel_diameter_m,
            "wheel_circumference_m": session.wheel_circumference_m,
            "created_at": session.created_at,
        },
        "rotation_trials": [asdict(t) for t in session.rotation_trials],
        "rotation_summary": session.rotation_summary(),
        "distance_trials": [asdict(t) for t in session.distance_trials],
        "distance_summary": session.distance_summary(),
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    with rot_csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "motor_id",
                "motor_label",
                "trial_index",
                "manual_revolutions",
                "start_count",
                "end_count",
                "delta_count",
                "counts_per_rev",
                "timestamp",
            ],
        )
        writer.writeheader()
        for t in session.rotation_trials:
            writer.writerow(asdict(t))

    with dist_csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "distance_m",
                "start_counts",
                "end_counts",
                "delta_counts",
                "avg_abs_counts",
                "counts_per_meter",
                "timestamp",
            ],
        )
        writer.writeheader()
        for t in session.distance_trials:
            row = asdict(t)
            row["start_counts"] = json.dumps(row["start_counts"])
            row["end_counts"] = json.dumps(row["end_counts"])
            row["delta_counts"] = json.dumps(row["delta_counts"])
            writer.writerow(row)

    return json_path, rot_csv_path, dist_csv_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive encoder calibration tool")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port, e.g. /dev/ttyACM0 or COM5")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--output-dir", default="calibration_logs", help="Directory to save logs")
    return parser.parse_args()


def print_intro() -> None:
    print("Encoder Calibration Tool")
    print("========================")
    print(f"Wheel diameter: {WHEEL_DIAMETER_IN:.3f} in")
    print(f"Wheel circumference: {WHEEL_CIRCUMFERENCE_M:.6f} m")
    print("\nThis tool supports two useful calibration modes:")
    print("1) Manual wheel-rotation calibration -> counts per wheel revolution")
    print("2) Straight-line floor calibration    -> counts per meter")
    print("\nRecommended order:")
    print("  a) verify motor IDs and encoder readback")
    print("  b) run manual rotation calibration for each wheel")
    print("  c) run straight-line calibration for the full robot")
    print("  d) use the averages in your main controller")


def main() -> int:
    args = parse_args()
    session = CalibrationSession(
        port=args.port,
        baud=args.baud,
        wheel_diameter_in=WHEEL_DIAMETER_IN,
        wheel_diameter_m=WHEEL_DIAMETER_M,
        wheel_circumference_m=WHEEL_CIRCUMFERENCE_M,
        created_at=datetime.now().isoformat(timespec="seconds"),
    )

    print_intro()
    print(f"\nOpening serial port {args.port} @ {args.baud}...")

    try:
        with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
            time.sleep(0.2)
            stop_all(ser)

            while True:
                print("\nMenu")
                print("----")
                print("1) Read all encoder states")
                print("2) Reset all encoders")
                print("3) Manual wheel-rotation calibration")
                print("4) Straight-line floor calibration")
                print("5) Show summaries")
                print("6) Live encoder monitor")
                print("7) Save logs and exit")
                print("8) Exit without saving")

                choice = prompt_int("Choose an option: ", valid={1, 2, 3, 4, 5, 6, 7, 8})

                if choice == 1:
                    states = read_all_motor_states(ser)
                    print("\nCurrent states")
                    print("--------------")
                    for s in states:
                        print(f"motor {s.motor_id} ({MOTOR_LABELS.get(s.motor_id, '?')}): count={s.count}, tps={s.tps:.3f}, rps={s.rps:.3f}")

                elif choice == 2:
                    states = reset_all_motors(ser)
                    print("\nAfter reset")
                    print("-----------")
                    for s in states:
                        print(f"motor {s.motor_id}: count={s.count}, tps={s.tps:.3f}, rps={s.rps:.3f}")

                elif choice == 3:
                    do_manual_rotation_calibration(ser, session)

                elif choice == 4:
                    do_straight_distance_calibration(ser, session)

                elif choice == 5:
                    print_rotation_summary(session)
                    print_distance_summary(session)

                elif choice == 6:
                    show_live_counts(ser)

                elif choice == 7:
                    stop_all(ser)
                    paths = save_logs(session, Path(args.output_dir))
                    print("\nSaved logs:")
                    for p in paths:
                        print(f"  {p}")
                    return 0

                elif choice == 8:
                    stop_all(ser)
                    print("Exiting without saving.")
                    return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 130
    except Exception as exc:
        print(f"\nERROR: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
