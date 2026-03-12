#!/usr/bin/env python3
import struct
import time
import serial

PORT = "/dev/ttyACM0"
BAUD = 1000000

HEADER = bytes([0xAA, 0x55])
FUNC_MOTOR = 0x03

def crc8_maxim(data: bytes) -> int:
    """
    Inferred from the protocol examples.
    CRC-8/MAXIM: poly 0x31, reflected, init 0x00, xorout 0x00
    """
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ 0x8C) & 0xFF  # reflected 0x31
            else:
                crc = (crc >> 1) & 0xFF
    return crc

def build_packet(function_code: int, payload: bytes) -> bytes:
    length = len(payload)
    body = bytes([function_code, length]) + payload
    checksum = crc8_maxim(body)
    return HEADER + body + bytes([checksum])

def send_packet(ser: serial.Serial, packet: bytes, label: str = ""):
    if label:
        print(f"\n{label}")
    print("TX:", packet.hex(" "))
    ser.write(packet)
    ser.flush()

def motor_run_single(motor_id: int, speed_rps: float) -> bytes:
    payload = bytes([0x00, motor_id]) + struct.pack("<f", speed_rps)
    return build_packet(FUNC_MOTOR, payload)

def motor_run_multi(motor_speeds: list[tuple[int, float]]) -> bytes:
    payload = bytes([0x01, len(motor_speeds)])
    for motor_id, speed in motor_speeds:
        payload += bytes([motor_id]) + struct.pack("<f", speed)
    return build_packet(FUNC_MOTOR, payload)

def motor_stop_single(motor_id: int) -> bytes:
    payload = bytes([0x02, motor_id])
    return build_packet(FUNC_MOTOR, payload)

def motor_stop_mask(mask: int) -> bytes:
    payload = bytes([0x03, mask & 0xFF])
    return build_packet(FUNC_MOTOR, payload)

def stop_all_four(ser: serial.Serial):
    # bit0->motor0, bit1->motor1, bit2->motor2, bit3->motor3
    send_packet(ser, motor_stop_mask(0x0F), "Stop all 4 motors")
    time.sleep(0.2)

def test_one_motor_at_a_time(ser: serial.Serial):
    stop_all_four(ser)

    # Try IDs 0..3 first because the stop-mask example explicitly references motor 0 and 2.
    for motor_id in range(4):
        send_packet(ser, motor_run_single(motor_id, 0.5),
                    f"Motor ID {motor_id}: +0.5 r/s for 2 s")
        time.sleep(2.0)

        send_packet(ser, motor_stop_single(motor_id),
                    f"Stop motor ID {motor_id}")
        time.sleep(1.0)

def test_reverse_direction(ser: serial.Serial):
    stop_all_four(ser)

    for motor_id in range(4):
        send_packet(ser, motor_run_single(motor_id, -0.5),
                    f"Motor ID {motor_id}: -0.5 r/s for 2 s")
        time.sleep(2.0)

        send_packet(ser, motor_stop_single(motor_id),
                    f"Stop motor ID {motor_id}")
        time.sleep(1.0)

def test_all_motors(ser: serial.Serial):
    stop_all_four(ser)

    send_packet(
        ser,
        motor_run_multi([(0, 0.4), (1, 0.4), (2, 0.4), (3, 0.4)]),
        "All 4 motors at +0.4 r/s"
    )
    time.sleep(2.0)
    stop_all_four(ser)

def test_left_right_split(ser: serial.Serial):
    stop_all_four(ser)

    # Example pattern for a 4-wheel differential robot:
    # left side forward, right side backward
    send_packet(
        ser,
        motor_run_multi([(0, 0.4), (1, 0.4), (2, -0.4), (3, -0.4)]),
        "Spin test: left pair +0.4, right pair -0.4"
    )
    time.sleep(2.0)
    stop_all_four(ser)

def main():
    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
        time.sleep(0.2)

        # Known-good example from PDF: motor 1 at -1 r/s
        pkt = motor_run_single(1, -1.0)
        send_packet(ser, pkt, "Known-good style test: motor 1 at -1.0 r/s")
        time.sleep(2.0)
        send_packet(ser, motor_stop_single(1), "Stop motor 1")
        time.sleep(1.0)

        test_one_motor_at_a_time(ser)
        test_reverse_direction(ser)
        test_all_motors(ser)
        test_left_right_split(ser)

        stop_all_four(ser)

if __name__ == "__main__":
    main()