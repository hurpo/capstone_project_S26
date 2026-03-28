#!/usr/bin/env python3
import serial
import struct
import time

PORT = "/dev/ttyACM0"
BAUD = 1000000

CRC8_TABLE = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
]


def crc8(data: bytes) -> int:
    check = 0
    for b in data:
        check = CRC8_TABLE[check ^ b]
    return check & 0xFF


def hex_bytes(data: bytes) -> str:
    return data.hex(" ").upper()


def send_packet(ser: serial.Serial, packet: bytes) -> None:
    print("TX:", hex_bytes(packet))
    ser.write(packet)
    ser.flush()


def build_buzzer_packet(freq_hz: int, on_ms: int, off_ms: int, count: int) -> bytes:
    body = bytes([0x02, 0x08]) + struct.pack("<HHHH", freq_hz, on_ms, off_ms, count)
    crc = crc8(body)
    return bytes([0xAA, 0x55]) + body + bytes([crc])


def build_led_packet(led_id: int, on_ms: int, off_ms: int, count: int) -> bytes:
    body = bytes([0x01, 0x07, led_id]) + struct.pack("<HHH", on_ms, off_ms, count)
    crc = crc8(body)
    return bytes([0xAA, 0x55]) + body + bytes([crc])


def ask_int(prompt: str, minimum: int, maximum: int) -> int:
    while True:
        raw = input(prompt).strip()
        try:
            value = int(raw)
            if minimum <= value <= maximum:
                return value
            print(f"Enter a value from {minimum} to {maximum}.")
        except ValueError:
            print("Enter a valid integer.")


def buzzer_menu(ser: serial.Serial) -> None:
    print("\nEnter buzzer values.")
    freq = ask_int("Frequency in Hz (10 to 20000): ", 10, 20000)
    on_ms = ask_int("Beep length in ms (0 to 65535): ", 0, 65535)
    off_ms = ask_int("Time between beeps in ms (0 to 65535): ", 0, 65535)
    count = ask_int("Number of beeps (0 to 65535, 0 may repeat forever in firmware): ", 0, 65535)

    packet = build_buzzer_packet(freq, on_ms, off_ms, count)
    send_packet(ser, packet)


def led_menu(ser: serial.Serial) -> None:
    print("\nEnter LED values.")
    led_id = ask_int("LED ID (usually 1): ", 0, 255)
    on_ms = ask_int("LED on time in ms (0 to 65535): ", 0, 65535)
    off_ms = ask_int("LED off time in ms (0 to 65535): ", 0, 65535)
    count = ask_int("Number of blinks (0 to 65535): ", 0, 65535)

    packet = build_led_packet(led_id, on_ms, off_ms, count)
    send_packet(ser, packet)


def raw_hex_menu(ser: serial.Serial) -> None:
    while True:
        raw = input("Paste full packet hex bytes: ").strip()
        try:
            packet = bytes.fromhex(raw)
            send_packet(ser, packet)
            return
        except ValueError:
            print("Invalid hex string.")


def presets_menu(ser: serial.Serial) -> None:
    print("\n1) Known-good buzzer example 1")
    print("2) Known-good buzzer example 2")
    print("3) Known-good LED example")
    choice = input("Selection: ").strip()

    if choice == "1":
        send_packet(ser, bytes.fromhex("AA 55 02 08 78 05 64 00 64 00 05 00 F0"))
    elif choice == "2":
        send_packet(ser, bytes.fromhex("AA 55 02 08 E8 03 F4 01 2C 01 0A 00 8B"))
    elif choice == "3":
        send_packet(ser, bytes.fromhex("AA 55 01 07 01 64 00 64 00 05 00 37"))
    else:
        print("Invalid selection.")


def main() -> None:
    print(f"Opening {PORT} at {BAUD} baud...")
    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
        time.sleep(0.2)

        while True:
            print("\n=== Hiwonder Control Board Terminal ===")
            print("1) Custom buzzer packet")
            print("2) Custom LED packet")
            print("3) Send known-good preset")
            print("4) Send raw hex packet")
            print("5) Quit")

            choice = input("Selection: ").strip()

            if choice == "1":
                buzzer_menu(ser)
            elif choice == "2":
                led_menu(ser)
            elif choice == "3":
                presets_menu(ser)
            elif choice == "4":
                raw_hex_menu(ser)
            elif choice == "5":
                print("Exiting.")
                break
            else:
                print("Invalid selection.")


if __name__ == "__main__":
    main()