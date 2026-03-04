from __future__ import annotations

import sys
import time

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
except ImportError:
    print("Missing libraries. Install with:")
    print("  sudo pip3 install adafruit-circuitpython-pca9685")
    sys.exit(1)

I2C_ADDRESS = 0x40
CHANNEL = 2
FREQ_HZ = 50

# Typical SG90 calibration starting points (not checked)
MIN_US = 500.0
MAX_US = 2500.0

MAX_ANGLE_DEG = 180.0

CLOSE_DEG = 80.0

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def us_to_duty_16bit(pulse_us: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / freq_hz
    duty_fraction = clamp(pulse_us / period_us, 0.0, 1.0)
    return int(duty_fraction * 65535.0)


def angle_to_us(angle_deg: float, min_us: float, max_us: float, max_angle_deg: float) -> float:
    angle_deg = clamp(angle_deg, 0.0, max_angle_deg)
    return min_us + (angle_deg / max_angle_deg) * (max_us - min_us)


class SG90Servo:
    def __init__(self, address=I2C_ADDRESS, channel=CHANNEL, freq_hz=FREQ_HZ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq_hz

        self.freq_hz = freq_hz
        self.ch = self.pca.channels[channel]

        self.min_us = float(MIN_US)
        self.max_us = float(MAX_US)
        self.max_angle_deg = float(MAX_ANGLE_DEG)

        self.last_commanded_deg = CLOSE_DEG
        self.set_angle(CLOSE_DEG)

    def set_pulse_us(self, pulse_us: float) -> None:
        self.ch.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def set_angle(self, angle_deg: float) -> float:
        angle_deg = clamp(angle_deg, 0.0, self.max_angle_deg)
        pulse = angle_to_us(angle_deg, self.min_us, self.max_us, self.max_angle_deg)
        self.set_pulse_us(pulse)
        self.last_commanded_deg = angle_deg
        return pulse

    def close(self) -> float:
        return self.set_angle(CLOSE_DEG)

    def open(self) -> float:
        return self.set_angle(15.0)

    def deinit(self) -> None:
        self.pca.deinit()


def print_help():
    print("""
Commands:
  close                  -> go to 80°
  open                   -> go to 15°
  angle <deg>            -> go to an absolute angle (0..180)
  range <min_us> <max_us>-> calibrate pulse endpoints (e.g., range 600 2400)
  info                   -> show current settings
  help                   -> show this help
  quit/exit              -> exit
""")


def main():
    servo = SG90Servo()
    print("SG90 180° servo controller (PCA9685 ch2). Type 'help'.")

    try:
        while True:
            cmdline = input("> ").strip()
            if not cmdline:
                continue

            parts = cmdline.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit"):
                break

            if cmd == "help":
                print_help()

            elif cmd == "info":
                print(f"addr=0x{I2C_ADDRESS:02X}, ch={CHANNEL}, freq={FREQ_HZ}Hz")
                print(f"range: min_us={servo.min_us}, max_us={servo.max_us}, max_angle={servo.max_angle_deg}")
                print(f"last_commanded_deg={servo.last_commanded_deg}")

            elif cmd == "range":
                if len(parts) < 3:
                    print("Usage: range <min_us> <max_us>")
                    continue
                servo.min_us = float(parts[1])
                servo.max_us = float(parts[2])
                print(f"Updated range: min_us={servo.min_us}, max_us={servo.max_us}")

            elif cmd == "close":
                pulse = servo.close()
                print(f"Closed at 80° (pulse={pulse:.0f}us)")

            elif cmd == "open":
                pulse = servo.open()
                print(f"opened at 15° (pulse={pulse:.0f}us)")

            elif cmd == "angle":
                if len(parts) < 2:
                    print("Usage: angle <deg>")
                    continue
                deg = float(parts[1])
                pulse = servo.set_angle(deg)
                print(f"Set angle to {deg:.1f}° (pulse={pulse:.0f}us)")

            else:
                print("Unknown command. Type 'help'.")

    except KeyboardInterrupt:
        print("\nExiting (Ctrl+C).")
    finally:
        servo.deinit()


if __name__ == "__main__":
    main()