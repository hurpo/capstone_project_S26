#!/usr/bin/env python3
"""
PCA9685 positional servo controller for a 270° servo
Raspberry Pi 4B -> PCA9685 over I2C, output channel 0

Behavior:
- 135° = close
- 270° = open

Commands:
  open                    -> move to 270°
  close                   -> move to 135°
  center                  -> move to 135°
  angle <deg>             -> move to any angle from 0..270
  pulse <us>              -> set direct pulse width
  range <min_us> <max_us> -> calibrate servo pulse range
  info                    -> show current settings
  help                    -> show commands
  quit/exit               -> exit
"""

import sys
import time

try:
    from adafruit_pca9685 import PCA9685
    import board
    import busio
except ImportError:
    print("Missing libraries. Install with:")
    print("  sudo pip3 install --break-system-packages adafruit-blinka adafruit-circuitpython-pca9685")
    sys.exit(1)

# ---------------------------
# Config
# ---------------------------
I2C_ADDRESS = 0x40
CHANNEL = 0
PWM_FREQUENCY_HZ = 50

# Starting calibration values
MIN_US_DEFAULT = 500
MAX_US_DEFAULT = 2500
MAX_ANGLE_DEG = 270.0

# Requested positions
CLOSE_ANGLE_DEG = 135.0
OPEN_ANGLE_DEG = 270.0


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def us_to_duty_16bit(microseconds: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / freq_hz
    duty_fraction = microseconds / period_us
    duty_fraction = clamp(duty_fraction, 0.0, 1.0)
    return int(duty_fraction * 65535.0)


def angle_to_us(angle_deg: float, min_us: float, max_us: float, max_angle_deg: float) -> float:
    angle_deg = clamp(angle_deg, 0.0, max_angle_deg)
    span = max_us - min_us
    return min_us + (angle_deg / max_angle_deg) * span


class Servo270:
    def __init__(self, address=I2C_ADDRESS, channel=CHANNEL, freq_hz=PWM_FREQUENCY_HZ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq_hz

        self.freq_hz = freq_hz
        self.channel = channel
        self.out = self.pca.channels[channel]

        self.min_us = float(MIN_US_DEFAULT)
        self.max_us = float(MAX_US_DEFAULT)
        self.max_angle_deg = float(MAX_ANGLE_DEG)

        # Start at close position = 135°
        self.set_angle(CLOSE_ANGLE_DEG)

    def set_pulse_us(self, pulse_us: float):
        period_us = 1_000_000.0 / self.freq_hz
        pulse_us = clamp(pulse_us, 0.0, period_us)
        self.out.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def set_angle(self, angle_deg: float) -> float:
        pulse = angle_to_us(angle_deg, self.min_us, self.max_us, self.max_angle_deg)
        self.set_pulse_us(pulse)
        return pulse

    def center(self) -> float:
        return self.set_angle(CLOSE_ANGLE_DEG)

    def close(self) -> float:
        return self.set_angle(CLOSE_ANGLE_DEG)

    def open(self) -> float:
        return self.set_angle(OPEN_ANGLE_DEG)

    def deinit(self):
        self.pca.deinit()


def print_help():
    print("""
Servo Commands:
  open                    -> move to 270° (open)
  close                   -> move to 135° (close)
  center                  -> same as close (135°)
  angle <deg>             -> command 0..270 degrees
  pulse <us>              -> set a specific pulse width in microseconds
  range <min_us> <max_us> -> set mapping pulse endpoints
  info                    -> show current settings
  help                    -> show this help
  quit/exit               -> exit
""")


def main():
    servo = Servo270()
    print("PCA9685 servo controller started on channel 0.")
    print("Servo initialized to 135° (close). Type 'help'.")

    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit"):
                break

            elif cmd == "help":
                print_help()

            elif cmd == "info":
                print(f"addr=0x{I2C_ADDRESS:02X}, channel={servo.channel}, freq={servo.freq_hz}Hz")
                print(f"max_angle={servo.max_angle_deg}°, min_us={servo.min_us}, max_us={servo.max_us}")
                print(f"close={CLOSE_ANGLE_DEG}°, open={OPEN_ANGLE_DEG}°")

            elif cmd == "range":
                if len(parts) < 3:
                    print("Usage: range <min_us> <max_us>")
                    continue
                servo.min_us = float(parts[1])
                servo.max_us = float(parts[2])
                print(f"Set range: min_us={servo.min_us}, max_us={servo.max_us}")

            elif cmd == "pulse":
                if len(parts) < 2:
                    print("Usage: pulse <us>")
                    continue
                us = float(parts[1])
                servo.set_pulse_us(us)
                print(f"Pulse set to {int(us)} us")

            elif cmd == "angle":
                if len(parts) < 2:
                    print("Usage: angle <deg>")
                    continue
                deg = float(parts[1])
                pulse = servo.set_angle(deg)
                print(f"Commanded {deg:.1f}° -> {pulse:.0f} us")

            elif cmd == "center":
                pulse = servo.center()
                print(f"Moved to 135° (close) -> {pulse:.0f} us")

            elif cmd == "close":
                pulse = servo.close()
                print(f"Moved to 135° (close) -> {pulse:.0f} us")

            elif cmd == "open":
                pulse = servo.open()
                print(f"Moved to 270° (open) -> {pulse:.0f} us")

            else:
                print("Unknown command. Type 'help'.")

    except KeyboardInterrupt:
        print("\nExiting (Ctrl+C).")
    finally:
        servo.deinit()


if __name__ == "__main__":
    main()