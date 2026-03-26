#!/usr/bin/env python3
"""
PCA9685 positional servo controller (270° mapping)
Raspberry Pi 4B -> PCA9685 over I2C, output channel 0.

Commands:
  angle <deg>              (0..270)
  center                   (135)
  range <min_us> <max_us>  (calibrate endpoints)
  pulse <us>               (direct pulse)
  sweep <min_us> <max_us>  (slow pulse sweep)
  info
  help
  quit/exit
"""

import sys
import time

try:
    from adafruit_pca9685 import PCA9685
    import board
    import busio
except ImportError:
    print("Missing libraries. Install with:")
    print("  sudo pip3 install adafruit-circuitpython-pca9685")
    sys.exit(1)

# ---------------------------
# Config
# ---------------------------
I2C_ADDRESS = 0x40
CHANNEL = 1
PWM_FREQUENCY_HZ = 50

# Typical starting calibration range (adjust for your servo)
MIN_US_DEFAULT = 500
MAX_US_DEFAULT = 2500

MAX_ANGLE_DEG = 270

SWEEP_STEP_US = 10
SWEEP_DELAY_S = 0.02


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

        # Start centered (135°)
        self.set_angle(self.max_angle_deg / 2)

    def set_pulse_us(self, pulse_us: float):
        period_us = 1_000_000.0 / self.freq_hz
        pulse_us = clamp(pulse_us, 0.0, period_us)
        self.out.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def set_angle(self, angle_deg: float) -> float:
        pulse = angle_to_us(angle_deg, self.min_us, self.max_us, self.max_angle_deg)
        self.set_pulse_us(pulse)
        return pulse

    def center(self) -> float:
        return self.set_angle(self.max_angle_deg / 2)

    def sweep(self, start_us: float, end_us: float, step_us: int = SWEEP_STEP_US, delay_s: float = SWEEP_DELAY_S):
        start_us = int(start_us)
        end_us = int(end_us)
        step = abs(int(step_us)) if end_us >= start_us else -abs(int(step_us))

        us = start_us
        while (step > 0 and us <= end_us) or (step < 0 and us >= end_us):
            self.set_pulse_us(us)
            time.sleep(delay_s)
            us += step

    def deinit(self):
        self.pca.deinit()


def print_help():
    print("""
270° Servo Commands:
  angle <deg>              -> command 0..270 degrees
  center                   -> go to 135 degrees
  range <min_us> <max_us>  -> set mapping pulse endpoints (calibration)
  pulse <us>               -> set a specific pulse width in microseconds
  sweep <min_us> <max_us>  -> sweep pulses slowly (observe end stops)
  info                     -> show current settings
  help                     -> show this help
  quit/exit                -> exit

Tips:
- Start with a conservative sweep: sweep 700 2300
- Then calibrate range so 0° and 270° land where you want:
  range 800 2200
""")


def main():
    servo = Servo270()
    print("PCA9685 270° servo controller (channel 0). Type 'help'.")

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
                print(f"max_angle={servo.max_angle_deg}°  min_us={servo.min_us}  max_us={servo.max_us}")

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
                print(f"Centered at 135° -> {pulse:.0f} us")

            elif cmd == "sweep":
                if len(parts) < 3:
                    print("Usage: sweep <min_us> <max_us>")
                    continue
                a = float(parts[1])
                b = float(parts[2])
                print("Sweeping... stop if it binds.")
                servo.sweep(a, b)
                servo.sweep(b, a)
                print("Sweep complete.")

            else:
                print("Unknown command. Type 'help'.")

    except KeyboardInterrupt:
        print("\nExiting (Ctrl+C).")
    finally:
        servo.deinit()


if __name__ == "__main__":
    main()