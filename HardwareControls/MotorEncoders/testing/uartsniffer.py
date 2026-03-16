#!/usr/bin/env python3
import struct
import time
import serial

PORT = "/dev/ttyACM0"
BAUD = 1000000
HEADER = bytes([0xAA, 0x55])
FUNC_MOTOR = 0x03

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
    checksum = crc8_maxim(body)
    return HEADER + body + bytes([checksum])

def motor_run_single(motor_id: int, speed_rps: float) -> bytes:
    payload = bytes([0x00, motor_id]) + struct.pack("<f", speed_rps)
    return build_packet(FUNC_MOTOR, payload)

def motor_stop_single(motor_id: int) -> bytes:
    payload = bytes([0x02, motor_id])
    return build_packet(FUNC_MOTOR, payload)

def dump_rx(ser, seconds=2.0):
    t0 = time.time()
    while time.time() - t0 < seconds:
        n = ser.in_waiting
        if n:
            data = ser.read(n)
            print(f"RX ({len(data)} bytes): {data.hex(' ')}")
        time.sleep(0.01)

def main():
    with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
        time.sleep(0.2)

        # clear stale data
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        pkt = motor_run_single(1, 1.0)
        print("TX:", pkt.hex(" "))
        ser.write(pkt)
        ser.flush()

        print("Listening while motor runs...")
        dump_rx(ser, seconds=3.0)

        pkt = motor_stop_single(1)
        print("TX:", pkt.hex(" "))
        ser.write(pkt)
        ser.flush()

        print("Listening after stop...")
        dump_rx(ser, seconds=1.0)

if __name__ == "__main__":
    main()