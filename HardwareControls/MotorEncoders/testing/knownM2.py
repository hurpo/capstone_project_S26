import serial
import struct
import time

PORT = "/dev/ttyACM0"
BAUD = 1000000

ser = serial.Serial(PORT, BAUD, timeout=0.2)
time.sleep(2.0)

def send_raw(hex_string: str, label: str = ""):
    pkt = bytes.fromhex(hex_string)
    if label:
        print(label)
    print("TX:", pkt.hex(" "))
    ser.write(pkt)
    ser.flush()

# Known-good packets you already verified:
RUN_MOTOR1_NEG1 = "AA5503060001000080BFDA"
STOP_MOTOR1     = "AA550302020108"

print("Testing exact known-good packet...")
send_raw(RUN_MOTOR1_NEG1, "Run motor with exact known-good packet")
time.sleep(3)
send_raw(STOP_MOTOR1, "Stop motor with exact known-good packet")
time.sleep(1)

ser.close()