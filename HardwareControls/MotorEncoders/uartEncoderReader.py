import struct
import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 1000000

HEADER = b'\xAA\x55'

def crc8_maxim(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = ((crc >> 1) ^ 0x8C) & 0xFF
            else:
                crc >>= 1
    return crc


def build_packet(func, payload):
    body = bytes([func, len(payload)]) + payload
    crc = crc8_maxim(body)
    return HEADER + body + bytes([crc])


def request_encoders(ser):
    pkt = build_packet(0x04, b'\x00')
    ser.write(pkt)
    ser.flush()


def read_encoder_packet(ser):

    header = ser.read(2)

    if header != b'\xAA\x55':
        return None

    func = ser.read(1)[0]
    length = ser.read(1)[0]

    data = ser.read(length)
    crc = ser.read(1)

    if func != 0x04:
        return None

    encoders = struct.unpack("<iiii", data)

    return encoders


with serial.Serial(PORT, BAUD, timeout=0.1) as ser:

    while True:

        request_encoders(ser)

        enc = read_encoder_packet(ser)

        if enc:
            print("Encoder counts:", enc)

        time.sleep(0.1)