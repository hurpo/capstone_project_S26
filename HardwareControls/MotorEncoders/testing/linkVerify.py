#!/usr/bin/env python3
import serial
import time

PORT = "/dev/ttyACM0"   # change if needed
BAUD = 1000000

def send_hex(ser, hex_string: str):
    packet = bytes.fromhex(hex_string)
    print("TX:", packet.hex(" "))
    ser.write(packet)
    ser.flush()

def play_beep():
    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
        time.sleep(0.2)
        send_hex(ser, "AA 55 02 08 78 05 64 00 64 00 05 00 F0")

def play_hw_error():
    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
        time.sleep(1)
        send_hex(ser, "AA 55 02 08 78 05 64 00 64 00 05 00 F0")
        send_hex(ser, "AA 55 01 07 01 64 00 64 00 05 00 37")
        time.sleep(1)
        send_hex(ser, "AA 55 02 08 78 05 64 00 64 00 05 00 F0")
        send_hex(ser, "AA 55 01 07 01 64 00 64 00 05 00 37")
        time.sleep(1)
        send_hex(ser, "AA 55 02 08 78 05 64 00 64 00 05 00 F0")
        send_hex(ser, "AA 55 01 07 01 64 00 64 00 05 00 37")

def main():
    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
        time.sleep(0.2)

        # LED blink 5 times, 100 ms on, 100 ms off
        # From the PDF examples:
        # AA 55 01 07 01 64 00 64 00 05 00 37
        send_hex(ser, "AA 55 01 07 01 64 00 64 00 05 00 37")
        time.sleep(0.1)

        # Buzzer 5 times at 1400 Hz, 100 ms on, 100 ms off
        # From the PDF examples:
        # AA 55 02 08 78 05 64 00 64 00 05 00 F0
        send_hex(ser, "AA 55 02 08 78 05 64 00 64 00 05 00 F0")

if __name__ == "__main__":
    main()