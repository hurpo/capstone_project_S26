#!/usr/bin/env python3
"""
Verify Hiwonder ROS Robot Control firmware motor + encoder communication.

Tests:
1) Read all encoder states
2) Reset all encoder counts
3) Run one motor for a short time
4) Read all encoder states again
5) Report whether the selected motor count changed

Expected firmware packet formats:
- request read one motor:   FUNC=0x03, payload=[0x10, motor_id]
- request read all motors:  FUNC=0x03, payload=[0x11]
- request reset one motor:  FUNC=0x03, payload=[0x12, motor_id]
- request reset all motors: FUNC=0x03, payload=[0x13]

Response payloads:
Single:
    [cmd, motor_id, count:int64_le, tps:float_le, rps:float_le]

All:
    [cmd, motor_num,
        motor_id, count:int64_le, tps:float_le, rps:float_le,
        ...
    ]
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
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


@dataclass
class MotorState:
    motor_id: int
    count: int
    tps: float
    rps: float


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
    """
    Reads one protocol packet:
      AA 55 FUNC LEN PAYLOAD CRC
    Returns full packet bytes or None on timeout.
    """
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
                state = 0
                buf.clear()
            continue

        if state == 2:
            buf.append(val)  # function
            state = 3
            continue

        if state == 3:
            buf.append(val)  # length
            payload_len = val
            remaining = payload_len + 1  # payload + crc
            tail = ser.read(remaining)
            if len(tail) != remaining:
                return None
            buf.extend(tail)
            return bytes(buf)

    return None


def validate_packet(packet: bytes) -> tuple[int, bytes]:
    if len(packet) < 5:
        raise ValueError("Packet too short")

    if packet[0:2] != HEADER:
        raise ValueError("Bad header")

    function_code = packet[2]
    length = packet[3]
    payload = packet[4:4 + length]
    rx_crc = packet[4 + length]

    body = packet[2:4 + length]
    calc_crc = crc8_maxim(body)
    if rx_crc != calc_crc:
        raise ValueError(
            f"Bad CRC: rx=0x{rx_crc:02X}, calc=0x{calc_crc:02X}"
        )

    return function_code, payload


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
    expected_len = 1 + 1 + 8 + 4 + 4
    if len(payload) != expected_len:
        raise ValueError(
            f"Single motor response payload length mismatch: "
            f"got {len(payload)}, expected {expected_len}"
        )

    cmd = payload[0]
    if cmd not in (CMD_ENCODER_READ_ONE, CMD_ENCODER_RESET_ONE):
        raise ValueError(f"Unexpected single response cmd: 0x{cmd:02X}")

    motor_id = payload[1]
    count = struct.unpack_from("<q", payload, 2)[0]
    tps = struct.unpack_from("<f", payload, 10)[0]
    rps = struct.unpack_from("<f", payload, 14)[0]

    return MotorState(motor_id=motor_id, count=count, tps=tps, rps=rps)


def parse_all_motor_payload(payload: bytes) -> list[MotorState]:
    if len(payload) < 2:
        raise ValueError("All motors response payload too short")

    cmd = payload[0]
    if cmd not in (CMD_ENCODER_READ_ALL, CMD_ENCODER_RESET_ALL):
        raise ValueError(f"Unexpected all response cmd: 0x{cmd:02X}")

    motor_num = payload[1]
    offset = 2
    entry_size = 1 + 8 + 4 + 4
    expected_len = 2 + motor_num * entry_size

    if len(payload) != expected_len:
        raise ValueError(
            f"All motors response payload length mismatch: "
            f"got {len(payload)}, expected {expected_len}"
        )

    motors: list[MotorState] = []
    for _ in range(motor_num):
        motor_id = payload[offset]
        count = struct.unpack_from("<q", payload, offset + 1)[0]
        tps = struct.unpack_from("<f", payload, offset + 9)[0]
        rps = struct.unpack_from("<f", payload, offset + 13)[0]
        motors.append(MotorState(motor_id=motor_id, count=count, tps=tps, rps=rps))
        offset += entry_size

    return motors


def transact(ser: serial.Serial, packet: bytes, timeout_s: float = 1.0, label: str = "") -> tuple[int, bytes, bytes]:
    ser.reset_input_buffer()
    send_packet(ser, packet, label=label)

    rx = read_exact_packet(ser, timeout_s=timeout_s)
    if rx is None:
        raise TimeoutError("Timed out waiting for response packet")

    print("RX:", hexdump(rx))
    func, payload = validate_packet(rx)
    return func, payload, rx


def print_motor_states(states: list[MotorState], title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))
    for s in states:
        print(
            f"motor {s.motor_id}: "
            f"count={s.count:>8d}   "
            f"tps={s.tps:>10.3f}   "
            f"rps={s.rps:>10.3f}"
        )


def stop_all_four(ser: serial.Serial) -> None:
    send_packet(ser, motor_stop_mask(0x0F), "Stop all 4 motors")
    time.sleep(0.25)


def verify_encoder_comm(
    ser: serial.Serial,
    test_motor: int,
    speed_rps: float,
    run_time: float,
) -> int:
    # Read initial state
    func, payload, _ = transact(ser, encoder_read_all(), label="Read all encoders (initial)")
    if func != FUNC_MOTOR:
        raise RuntimeError(f"Unexpected function code in response: 0x{func:02X}")
    before = parse_all_motor_payload(payload)
    print_motor_states(before, "Initial motor states")

    # Reset all
    func, payload, _ = transact(ser, encoder_reset_all(), label="Reset all encoders")
    if func != FUNC_MOTOR:
        raise RuntimeError(f"Unexpected function code in reset response: 0x{func:02X}")
    reset_states = parse_all_motor_payload(payload)
    print_motor_states(reset_states, "States immediately after reset")

    # Confirm reset with another read
    func, payload, _ = transact(ser, encoder_read_all(), label="Read all encoders after reset")
    after_reset = parse_all_motor_payload(payload)
    print_motor_states(after_reset, "Post-reset motor states")

    # Run one motor
    send_packet(
        ser,
        motor_run_single(test_motor, speed_rps),
        label=f"Run motor {test_motor} at {speed_rps:.3f} rps for {run_time:.2f}s",
    )
    time.sleep(run_time)
    send_packet(ser, motor_stop_single(test_motor), label=f"Stop motor {test_motor}")
    time.sleep(0.25)

    # Read back
    func, payload, _ = transact(ser, encoder_read_all(), label="Read all encoders after motor run")
    after_run = parse_all_motor_payload(payload)
    print_motor_states(after_run, "Motor states after run")

    by_id_before = {m.motor_id: m for m in after_reset}
    by_id_after = {m.motor_id: m for m in after_run}

    target_before = by_id_before[test_motor]
    target_after = by_id_after[test_motor]
    delta = target_after.count - target_before.count

    print("\nVerification result")
    print("-------------------")
    print(f"Test motor: {test_motor}")
    print(f"Count before run: {target_before.count}")
    print(f"Count after run : {target_after.count}")
    print(f"Delta count     : {delta}")
    print(f"Reported rps    : {target_after.rps:.3f}")
    print(f"Reported tps    : {target_after.tps:.3f}")

    # A simple pass/fail rule:
    # If the motor physically moved and firmware is working, delta should not be zero.
    if delta == 0:
        print("\nFAIL: encoder count did not change for the test motor.")
        print("Possible causes:")
        print("- wrong motor_id selected")
        print("- motor did not physically move")
        print("- encoder wiring/config for that motor is wrong")
        print("- firmware patch is not installed or packet layout differs")
        return 1

    print("\nPASS: encoder count changed, so encoder communication is working.")
    return 0


def read_single_demo(ser: serial.Serial, motor_id: int) -> None:
    func, payload, _ = transact(ser, encoder_read_one(motor_id), label=f"Read single motor {motor_id}")
    if func != FUNC_MOTOR:
        raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
    state = parse_single_motor_payload(payload)
    print_motor_states([state], f"Single motor {motor_id} state")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify Hiwonder encoder firmware over serial."
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port, e.g. /dev/ttyACM0 or COM5")
    parser.add_argument("--baud", type=int, default=1000000, help="Serial baud rate")
    parser.add_argument("--motor", type=int, default=0, choices=[0, 1, 2, 3], help="Motor ID to test")
    parser.add_argument("--speed", type=float, default=0.5, help="Motor speed command in rps")
    parser.add_argument("--time", type=float, default=2.0, help="Run time in seconds")
    parser.add_argument("--timeout", type=float, default=1.0, help="Packet receive timeout in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("Opening serial port...")
    print(f"  port = {args.port}")
    print(f"  baud = {args.baud}")

    try:
        with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
            time.sleep(0.2)

            stop_all_four(ser)
            read_single_demo(ser, args.motor)
            rc = verify_encoder_comm(
                ser=ser,
                test_motor=args.motor,
                speed_rps=args.speed,
                run_time=args.time,
            )
            stop_all_four(ser)
            return rc

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 130
    except Exception as exc:
        print(f"\nERROR: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())