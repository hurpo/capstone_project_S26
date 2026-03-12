#!/usr/bin/env python3
import serial
import struct
import time

PORT = "/dev/ttyACM0"
BAUD = 1000000

def checksum(func: int, length: int, data: bytes) -> int:
    return (func + length + sum(data)) & 0xFF

def build_buzzer_packet(freq_hz: int, beep_ms: int, silence_ms: int, cycles: int) -> bytes:
    func = 0x02
    length = 0x08
    data = struct.pack("<HHHH", freq_hz, beep_ms, silence_ms, cycles)
    crc = checksum(func, length, data)
    return bytes([0xAA, 0x55, func, length]) + data + bytes([crc])

def send_packet(ser, pkt, label=""):
    if label:
        print(label)
    print("TX:", pkt.hex(" "))
    ser.write(pkt)
    ser.flush()

def play_note(ser, freq_hz: int, duration_ms: int):
    # Give the board a whole note as a single cycle
    pkt = build_buzzer_packet(freq_hz, duration_ms, 80, 1)
    send_packet(ser, pkt)
    time.sleep((duration_ms + 120) / 1000.0)  # extra slack

def main():
    # Disable extra flow-control features and give the board time to boot
    with serial.Serial(
        PORT,
        BAUD,
        timeout=0.1,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False
    ) as ser:
        # Some USB-serial devices reset the controller when the port opens
        time.sleep(2.0)

        # Clear any junk
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.2)

        # EXACT known-good packet from the PDF:
        # 1400 Hz, 100 ms on, 100 ms off, 5 cycles
        pkt1 = bytes.fromhex("AA 55 02 08 78 05 64 00 64 00 05 00 F0")
        send_packet(ser, pkt1, "Known-good test 1")
        time.sleep(2.0)

        # EXACT known-good packet from the PDF:
        # 1000 Hz, 500 ms on, 300 ms off, 10 cycles
        pkt2 = bytes.fromhex("AA 55 02 08 E8 03 F4 01 2C 01 0A 00 8B")
        send_packet(ser, pkt2, "Known-good test 2")
        time.sleep(8.5)

        # If both worked, now try a very simple high-pitch melody.
        # Keep everything in the buzzer's comfortable range.
        melody = [
            (1400, 220),
            (1600, 220),
            (1200, 220),
            (1600, 220),
            (1800, 350),
            (2000, 350),
            (1800, 500),

            (1400, 220),
            (1600, 220),
            (1200, 220),
            (1600, 220),
            (1400, 350),
            (1100, 350),
            (1000, 500),
        ]

        print("Starting melody...")
        for freq, dur in melody:
            play_note(ser, freq, dur)

if __name__ == "__main__":
    main()