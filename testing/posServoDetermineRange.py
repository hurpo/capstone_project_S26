#!/usr/bin/env python3
"""
Positional servo control via PCA9685 on Raspberry Pi (I2C).
- Channel 0 by default.
- Includes a safe pulse sweep to help you determine range/travel (e.g., 180 vs 270).
- Maps commanded angle -> pulse width based on configurable min/max pulse and max_angle.

NOTE on feedback:
A standard 3-wire servo does NOT report its angle. This script reports the *commanded* angle.
To measure actual angle, you need a feedback-capable servo (extra wire) + ADC, or an external sensor.
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
CHANNEL = 0
PWM_FREQUENCY_HZ = 50

# Default pulse range (microseconds) - you will calibrate these
MIN_US = 500
MAX_US = 2500

# Default assumed servo travel. Set to 180 or 270 after you determine it.
MAX_ANGLE_DEG = 180

# Sweep behavior
SWEEP_STEP_US = 10
SWEEP_DELAY_S = 0.02

# ---------------------------
# Helpers
# ---------------------------
def clamp(x, lo, hi):
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

# ---------------------------
# Controller
# ---------------------------
class PositionalServoPCA9685:
    def __init__(self, address=I2C_ADDRESS, channel=CHANNEL, freq_hz=PWM_FREQUENCY_HZ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq_hz
        self.freq_hz = freq_hz

        self.channel = channel
        self.out = self.pca.channels[channel]

        self.min_us = MIN_US
        self.max_us = MAX_US
        self.max_angle_deg = MAX_ANGLE_DEG

        # Move to mid on start
        self.set_angle(self.max_angle_deg / 2)

    def set_pulse_us(self, pulse_us: float):
        pulse_us = clamp(pulse_us, 0.0, 1_000_000.0 / self.freq_hz)  # clamp to period
        self.out.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def set_angle(self, angle_deg: float):
        pulse = angle_to_us(angle_deg, self.min_us, self.max_us, self.max_angle_deg)
        self.set_pulse_us(pulse)
        return pulse

    def sweep_us(self, start_us: float, end_us: float, step_us: int = SWEEP_STEP_US, delay_s: float = SWEEP_DELAY_S):
        start_us = int(start_us)
        end_us = int(end_us)
        step_us = abs(int(step_us)) if end_us >= start_us else -abs(int(step_us))

        us = start_us
        while (us <= end_us and step_us > 0) or (us >= end_us and step_us < 0):
            self.set_pulse_us(us)
            time.sleep(delay_s)
            us += step_us

    def deinit(self):
        try:
            # Hold last command (normal for servos)
            time.sleep(0.05)
        finally:
            self.pca.deinit()

# ---------------------------
# CLI
# ---------------------------
def print_help():
    print("""
Commands:
  angle <deg>              -> command angle (0..max_angle)
  maxangle <180|270>        -> set assumed travel
  range <min_us> <max_us>   -> set pulse range used for mapping (calibration)
  pulse <us>                -> directly set a pulse width (debug)
  center                    -> go to half of maxangle
  sweep <min_us> <max_us>   -> sweep pulses slowly so you can observe travel safely
  info                      -> show current calibration
  help                      -> show help
  quit/exit                 -> exit

Workflow to determine 180 vs 270:
  1) run: sweep 700 2300   (start narrower if you're cautious)
  2) observe whether travel is ~180° or closer to ~270°
  3) set: maxangle 180  (or 270)
  4) calibrate range with range <min_us> <max_us> so angle mapping matches endpoints
""")

def main():
    servo = PositionalServoPCA9685()
    print("PCA9685 positional servo controller (channel 0). Type 'help' for commands.")

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
                print(f"channel={servo.channel}, freq={servo.freq_hz}Hz")
                print(f"min_us={servo.min_us}, max_us={servo.max_us}, max_angle_deg={servo.max_angle_deg}")

            elif cmd == "maxangle":
                if len(parts) < 2:
                    print("Usage: maxangle <180|270>")
                    continue
                val = int(parts[1])
                if val not in (180, 270):
                    print("maxangle must be 180 or 270.")
                    continue
                servo.max_angle_deg = val
                print(f"Set max_angle_deg={val}")

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
                print(f"Commanded angle={deg:.1f}° -> pulse={pulse:.0f} us")

            elif cmd == "center":
                deg = servo.max_angle_deg / 2
                pulse = servo.set_angle(deg)
                print(f"Centered at {deg:.1f}° -> pulse={pulse:.0f} us")

            elif cmd == "sweep":
                if len(parts) < 3:
                    print("Usage: sweep <min_us> <max_us>")
                    continue
                a = float(parts[1])
                b = float(parts[2])
                print("Sweeping pulses... watch the horn and STOP if it binds.")
                servo.sweep_us(a, b)
                servo.sweep_us(b, a)
                print("Sweep complete.")

            else:
                print("Unknown command. Type 'help'.")

    except KeyboardInterrupt:
        print("\nExiting (Ctrl+C).")
    finally:
        servo.deinit()

if __name__ == "__main__":
    main()