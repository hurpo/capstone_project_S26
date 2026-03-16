#!/usr/bin/env python3
"""
Interactive encoder calibration tool.

This script now writes a shared robot_calibration.json file that MotorControl.py
can load directly, so the calibration and control workflows stay in sync.

All user-entered distances are in inches.
All internal math is converted to meters.
Commanded motor speed is in rev/s.
"""

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
from typing import Dict, Optional

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
DEFAULT_CAL_FILE = "robot_calibration.json"

FL = 0
RL = 1
FR = 2
RR = 3

DEFAULT_DIRECTION_SIGNS = {
    0: -1,  # left side reversed for forward motion
    1: -1,
    2: +1,
    3: +1,
}

DEFAULT_MAPPING = {
    0: "front_left",
    1: "rear_left",
    2: "front_right",
    3: "rear_right",
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
    distance_in: float
    distance_m: float
    end_counts: Dict[int, int]
    delta_counts: Dict[int, int]
    avg_abs_counts: float
    counts_per_meter: float
    implied_counts_per_rev: float
    estimated_speed_m_s: float
    timestamp: str


@dataclass
class CalibrationSession:
    port: str
    baud: int
    wheel_diameter_in: float
    half_length_in: float
    half_width_in: float
    motor_direction_signs: Dict[int, int]
    motor_mapping: Dict[int, str]
    created_at: str
    rotation_trials: list[RotationTrial] = field(default_factory=list)
    distance_trials: list[DistanceTrial] = field(default_factory=list)

    def rotation_summary(self) -> Dict[int, dict]:
        grouped: Dict[int, list[RotationTrial]] = {}
        for t in self.rotation_trials:
            grouped.setdefault(t.motor_id, []).append(t)

        out = {}
        for motor_id, trials in grouped.items():
            vals = [abs(t.counts_per_rev) for t in trials]
            out[motor_id] = {
                "motor_label": trials[0].motor_label,
                "num_trials": len(vals),
                "avg_counts_per_rev": sum(vals) / len(vals),
                "min_counts_per_rev": min(vals),
                "max_counts_per_rev": max(vals),
            }
        return out

    def distance_summary(self) -> dict:
        if not self.distance_trials:
            return {}
        vals = [t.counts_per_meter for t in self.distance_trials]
        implied = [t.implied_counts_per_rev for t in self.distance_trials]
        return {
            "num_trials": len(vals),
            "avg_counts_per_meter": sum(vals) / len(vals),
            "min_counts_per_meter": min(vals),
            "max_counts_per_meter": max(vals),
            "avg_implied_counts_per_rev": sum(implied) / len(implied),
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
    body = bytes([function_code, len(payload)]) + payload
    return HEADER + body + bytes([crc8_maxim(body)])


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
                buf[:] = b
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
    send_packet(ser, packet, label)
    rx = read_exact_packet(ser, timeout_s)
    if rx is None:
        raise TimeoutError("Timed out waiting for response")
    print("RX:", hexdump(rx))
    return validate_packet(rx)


def motor_run_single(motor_id: int, speed_rps: float) -> bytes:
    return build_packet(FUNC_MOTOR, bytes([CMD_MOTOR_RUN_SINGLE, motor_id]) + struct.pack("<f", speed_rps))


def motor_stop_single(motor_id: int) -> bytes:
    return build_packet(FUNC_MOTOR, bytes([CMD_MOTOR_STOP_SINGLE, motor_id]))


def motor_stop_mask(mask: int) -> bytes:
    return build_packet(FUNC_MOTOR, bytes([CMD_MOTOR_STOP_MASK, mask & 0xFF]))


def encoder_read_one(motor_id: int) -> bytes:
    return build_packet(FUNC_MOTOR, bytes([CMD_ENCODER_READ_ONE, motor_id]))


def encoder_read_all() -> bytes:
    return build_packet(FUNC_MOTOR, bytes([CMD_ENCODER_READ_ALL]))


def encoder_reset_one(motor_id: int) -> bytes:
    return build_packet(FUNC_MOTOR, bytes([CMD_ENCODER_RESET_ONE, motor_id]))


def encoder_reset_all() -> bytes:
    return build_packet(FUNC_MOTOR, bytes([CMD_ENCODER_RESET_ALL]))


def parse_single_motor_payload(payload: bytes) -> MotorState:
    motor_id = payload[1]
    count = struct.unpack_from("<q", payload, 2)[0]
    tps = struct.unpack_from("<f", payload, 10)[0]
    rps = struct.unpack_from("<f", payload, 14)[0]
    return MotorState(motor_id, count, tps, rps)


def parse_all_motor_payload(payload: bytes) -> list[MotorState]:
    motor_num = payload[1]
    offset = 2
    states = []
    for _ in range(motor_num):
        motor_id = payload[offset]
        count = struct.unpack_from("<q", payload, offset + 1)[0]
        tps = struct.unpack_from("<f", payload, offset + 9)[0]
        rps = struct.unpack_from("<f", payload, offset + 13)[0]
        states.append(MotorState(motor_id, count, tps, rps))
        offset += 17
    return states


def stop_all(ser: serial.Serial) -> None:
    send_packet(ser, motor_stop_mask(0x0F), "Stop all motors")
    time.sleep(0.15)


def read_motor(ser: serial.Serial, motor_id: int) -> MotorState:
    func, payload = transact(ser, encoder_read_one(motor_id), label=f"Read motor {motor_id}")
    if func != FUNC_MOTOR:
        raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
    return parse_single_motor_payload(payload)


def read_all(ser: serial.Serial) -> list[MotorState]:
    func, payload = transact(ser, encoder_read_all(), label="Read all motor states")
    if func != FUNC_MOTOR:
        raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
    return parse_all_motor_payload(payload)


def reset_motor(ser: serial.Serial, motor_id: int) -> MotorState:
    func, payload = transact(ser, encoder_reset_one(motor_id), label=f"Reset motor {motor_id}")
    if func != FUNC_MOTOR:
        raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
    return parse_single_motor_payload(payload)


def reset_all(ser: serial.Serial) -> list[MotorState]:
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
        print("\nNo straight-line data collected yet.")
        return
    print("\nStraight-line summary")
    print("---------------------")
    print(
        f"avg={s['avg_counts_per_meter']:.3f} counts/m over {s['num_trials']} trial(s), "
        f"range=[{s['min_counts_per_meter']:.3f}, {s['max_counts_per_meter']:.3f}]"
    )
    print(f"avg implied counts/rev: {s['avg_implied_counts_per_rev']:.3f}")


def do_manual_rotation_calibration(ser: serial.Serial, session: CalibrationSession) -> None:
    print("\nManual wheel rotation calibration")
    print("--------------------------------")
    print("Enter revolutions manually. Use 5 or 10 turns for better averaging.")

    while True:
        motor_id = prompt_int("Motor ID to calibrate (0-3): ", valid={0, 1, 2, 3})
        label = session.motor_mapping[motor_id]
        trials = prompt_int(f"How many trials for motor {motor_id} ({label})? ", min_value=1)

        for idx in range(1, trials + 1):
            print(f"\nTrial {idx}/{trials} for motor {motor_id} ({label})")
            reset_motor(ser, motor_id)
            input("Press Enter when ready to start manual rotation... ")
            start_state = read_motor(ser, motor_id)
            revs = prompt_float("How many exact wheel revolutions will you rotate? ", min_value=0.001)
            input("Rotate the wheel now, then press Enter when done... ")
            end_state = read_motor(ser, motor_id)
            delta = end_state.count - start_state.count
            counts_per_rev = abs(delta) / revs

            session.rotation_trials.append(
                RotationTrial(
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
            )
            print(f"Recorded counts/rev = {counts_per_rev:.6f}")

        print_rotation_summary(session)
        if not prompt_yes_no("Calibrate another motor manually?", default=False):
            break


def do_straight_distance_calibration(ser: serial.Serial, session: CalibrationSession) -> None:
    print("\nStraight-line drive calibration")
    print("-------------------------------")
    print("All user-entered distances are in inches.")
    speed_rps = prompt_float("Commanded motor speed in rev/s (e.g. 0.3): ", min_value=0.001)
    run_time = prompt_float("Open-loop run time in seconds (e.g. 1.5): ", min_value=0.01)
    trials = prompt_int("How many trials? ", min_value=1)

    wheel_diameter_m = session.wheel_diameter_in * 0.0254
    wheel_circumference_m = math.pi * wheel_diameter_m

    for idx in range(1, trials + 1):
        print(f"\nStraight-line trial {idx}/{trials}")
        reset_all(ser)
        input("Place robot at the start mark, then press Enter... ")

        send_packet(ser, motor_run_single(0, -speed_rps), label="Run motor 0")
        send_packet(ser, motor_run_single(1, -speed_rps), label="Run motor 1")
        send_packet(ser, motor_run_single(2, +speed_rps), label="Run motor 2")
        send_packet(ser, motor_run_single(3, +speed_rps), label="Run motor 3")
        time.sleep(run_time)
        stop_all(ser)

        states = read_all(ser)
        end_counts = {s.motor_id: s.count for s in states}
        delta_counts = end_counts.copy()

        print("\nCounts after run:")
        for s in states:
            print(f"  motor {s.motor_id}: count={s.count}, tps={s.tps:.3f}, rps={s.rps:.3f}")

        distance_in = prompt_float("Enter the ACTUAL measured straight distance traveled in inches: ", min_value=0.0001)
        distance_m = distance_in * 0.0254
        avg_abs_counts = sum(abs(v) for v in delta_counts.values()) / len(delta_counts)
        counts_per_meter = avg_abs_counts / distance_m
        implied_counts_per_rev = counts_per_meter * wheel_circumference_m
        estimated_speed_m_s = distance_m / run_time

        session.distance_trials.append(
            DistanceTrial(
                distance_in=distance_in,
                distance_m=distance_m,
                end_counts=end_counts,
                delta_counts=delta_counts,
                avg_abs_counts=avg_abs_counts,
                counts_per_meter=counts_per_meter,
                implied_counts_per_rev=implied_counts_per_rev,
                estimated_speed_m_s=estimated_speed_m_s,
                timestamp=datetime.now().isoformat(timespec="seconds"),
            )
        )

        print("\nRecorded straight-line trial")
        print("----------------------------")
        print(f"distance_in           = {distance_in:.6f}")
        print(f"distance_m            = {distance_m:.6f}")
        print(f"avg_abs_counts        = {avg_abs_counts:.6f}")
        print(f"counts_per_meter      = {counts_per_meter:.6f}")
        print(f"implied counts/rev    = {implied_counts_per_rev:.6f}")
        print(f"estimated speed       = {estimated_speed_m_s:.6f} m/s")

    print_distance_summary(session)


def save_logs_and_calibration(session: CalibrationSession, output_dir: Path, cal_file: Path) -> tuple[Path, Path, Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_log = output_dir / f"encoder_calibration_session_{ts}.json"
    rot_csv = output_dir / f"encoder_rotation_trials_{ts}.csv"
    dist_csv = output_dir / f"encoder_distance_trials_{ts}.csv"

    rotation_summary = session.rotation_summary()
    distance_summary = session.distance_summary()

    session_payload = {
        "session": {
            "port": session.port,
            "baud": session.baud,
            "wheel_diameter_in": session.wheel_diameter_in,
            "half_length_in": session.half_length_in,
            "half_width_in": session.half_width_in,
            "created_at": session.created_at,
        },
        "rotation_trials": [asdict(t) for t in session.rotation_trials],
        "rotation_summary": rotation_summary,
        "distance_trials": [asdict(t) for t in session.distance_trials],
        "distance_summary": distance_summary,
    }
    json_log.write_text(json.dumps(session_payload, indent=2), encoding="utf-8")

    with rot_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "motor_id", "motor_label", "trial_index", "manual_revolutions",
            "start_count", "end_count", "delta_count", "counts_per_rev", "timestamp"
        ])
        writer.writeheader()
        for t in session.rotation_trials:
            writer.writerow(asdict(t))

    with dist_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "distance_in", "distance_m", "end_counts", "delta_counts",
            "avg_abs_counts", "counts_per_meter", "implied_counts_per_rev",
            "estimated_speed_m_s", "timestamp"
        ])
        writer.writeheader()
        for t in session.distance_trials:
            row = asdict(t)
            row["end_counts"] = json.dumps(row["end_counts"])
            row["delta_counts"] = json.dumps(row["delta_counts"])
            writer.writerow(row)

    counts_per_rev = {}
    for motor_id in range(4):
        if motor_id in rotation_summary:
            counts_per_rev[str(motor_id)] = rotation_summary[motor_id]["avg_counts_per_rev"]

    cal_payload = {
        "wheel": {
            "diameter_in": session.wheel_diameter_in,
        },
        "robot_geometry": {
            "half_length_in": session.half_length_in,
            "half_width_in": session.half_width_in,
        },
        "motor_mapping": {str(k): v for k, v in session.motor_mapping.items()},
        "motor_direction_signs": {str(k): int(v) for k, v in session.motor_direction_signs.items()},
        "counts_per_revolution": counts_per_rev,
        "straight_line": {
            "counts_per_meter_avg": distance_summary.get("avg_counts_per_meter"),
            "implied_counts_per_rev": distance_summary.get("avg_implied_counts_per_rev"),
            "preferred_distance_model": "per_wheel_counts",
        },
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "source_log": str(json_log),
    }
    cal_file.write_text(json.dumps(cal_payload, indent=2), encoding="utf-8")
    return json_log, rot_csv, dist_csv, cal_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive encoder calibration tool")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--output-dir", default="calibration_logs", help="Directory to save logs")
    parser.add_argument("--calibration-file", default=DEFAULT_CAL_FILE, help="Shared calibration JSON file")
    return parser.parse_args()


def print_intro() -> None:
    print("Encoder Calibration Tool")
    print("========================")
    print("All distances entered by the user are in inches.")
    print("Internal math is converted to meters.")
    print("The saved robot_calibration.json file can be loaded directly by MotorControl.py.")


def main() -> int:
    args = parse_args()
    print_intro()

    wheel_diameter_in = prompt_float("Wheel diameter in inches [2.25 typical]: ", min_value=0.001)
    half_length_in = prompt_float("Robot half-length in inches (center to wheel center front/back): ", min_value=0.001)
    half_width_in = prompt_float("Robot half-width in inches (center to wheel center left/right): ", min_value=0.001)

    session = CalibrationSession(
        port=args.port,
        baud=args.baud,
        wheel_diameter_in=wheel_diameter_in,
        half_length_in=half_length_in,
        half_width_in=half_width_in,
        motor_direction_signs=DEFAULT_DIRECTION_SIGNS.copy(),
        motor_mapping=DEFAULT_MAPPING.copy(),
        created_at=datetime.now().isoformat(timespec="seconds"),
    )

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
                print("6) Save logs + shared robot_calibration.json")
                print("7) Exit without saving")

                choice = prompt_int("Choose an option: ", valid={1,2,3,4,5,6,7})

                if choice == 1:
                    states = read_all(ser)
                    for s in states:
                        print(f"motor {s.motor_id} ({session.motor_mapping[s.motor_id]}): count={s.count}, tps={s.tps:.3f}, rps={s.rps:.3f}")

                elif choice == 2:
                    states = reset_all(ser)
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
                    paths = save_logs_and_calibration(
                        session,
                        Path(args.output_dir),
                        Path(args.calibration_file),
                    )
                    print("\nSaved files:")
                    for p in paths:
                        print(f"  {p}")
                    return 0

                elif choice == 7:
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
