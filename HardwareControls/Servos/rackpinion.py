#!/usr/bin/env python3
"""
PCA9685 positional servo controller for a 270° servo
Raspberry Pi 4B -> PCA9685 over I2C, output channel 0.

Behavior:
- On startup, servo moves to 0° (lowered)
- 270° is treated as the raised position
- Methods are structured so they can be imported and used by other programs

Commands:
  lower                    -> move to 0°
  raise                    -> move to 270°
  angle <deg>              -> move to any angle from 0..270
  center                   -> move to 135°
  range <min_us> <max_us>  -> calibrate pulse endpoints
  pulse <us>               -> directly set pulse width
  sweep <start> <end>      -> sweep through angles in degrees
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
CHANNEL = 5
PWM_FREQUENCY_HZ = 50

# Starting calibration values for many 270° servos
MIN_US_DEFAULT = 500
MAX_US_DEFAULT = 2500

MIN_ANGLE_DEG = 0.0
MAX_ANGLE_DEG = 270.0
DEFAULT_START_ANGLE_DEG = 0.0   # lowered at startup
LOWERED_ANGLE_DEG = 0.0
RAISED_ANGLE_DEG = 270.0
CENTER_ANGLE_DEG = 135.0

SWEEP_STEP_DEG = 2.0
SWEEP_DELAY_S = 0.02


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def us_to_duty_16bit(microseconds: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / freq_hz
    duty_fraction = clamp(microseconds / period_us, 0.0, 1.0)
    return int(duty_fraction * 65535.0)


def angle_to_us(angle_deg: float, min_us: float, max_us: float,
                min_angle_deg: float, max_angle_deg: float) -> float:
    angle_deg = clamp(angle_deg, min_angle_deg, max_angle_deg)
    angle_span = max_angle_deg - min_angle_deg
    pulse_span = max_us - min_us
    return min_us + ((angle_deg - min_angle_deg) / angle_span) * pulse_span


class Servo270:
    """
    270-degree positional servo controller.

    Useful methods for other programs:
      - move_to_angle(angle_deg)
      - lower()
      - raise_up()
      - center()
      - set_pulse_us(pulse_us)
      - set_range(min_us, max_us)
      - sweep_angles(start_deg, end_deg)
      - deinit()
    """

    def __init__(
        self,
        address: int = I2C_ADDRESS,
        channel: int = CHANNEL,
        freq_hz: float = PWM_FREQUENCY_HZ,
        min_us: float = MIN_US_DEFAULT,
        max_us: float = MAX_US_DEFAULT,
        start_angle_deg: float = DEFAULT_START_ANGLE_DEG,
    ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq_hz

        self.freq_hz = freq_hz
        self.channel = channel
        self.out = self.pca.channels[channel]

        self.min_us = float(min_us)
        self.max_us = float(max_us)
        self.min_angle_deg = float(MIN_ANGLE_DEG)
        self.max_angle_deg = float(MAX_ANGLE_DEG)
        self.current_angle_deg = None

        # Start at lowered position (0°)
        self.move_to_angle(start_angle_deg)

    def set_pulse_us(self, pulse_us: float) -> None:
        period_us = 1_000_000.0 / self.freq_hz
        pulse_us = clamp(pulse_us, 0.0, period_us)
        self.out.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def move_to_angle(self, angle_deg: float) -> float:
        angle_deg = clamp(angle_deg, self.min_angle_deg, self.max_angle_deg)
        pulse_us = angle_to_us(
            angle_deg,
            self.min_us,
            self.max_us,
            self.min_angle_deg,
            self.max_angle_deg,
        )
        self.set_pulse_us(pulse_us)
        self.current_angle_deg = angle_deg
        return pulse_us

    def lower(self) -> float:
        """Move servo to 0° (lowered)."""
        return self.move_to_angle(LOWERED_ANGLE_DEG)

    def raise_up(self) -> float:
        """Move servo to 270° (raised)."""
        return self.move_to_angle(RAISED_ANGLE_DEG)

    def center(self) -> float:
        """Move servo to 135°."""
        return self.move_to_angle(CENTER_ANGLE_DEG)

    def set_range(self, min_us: float, max_us: float) -> None:
        self.min_us = float(min_us)
        self.max_us = float(max_us)

    def sweep_angles(
        self,
        start_deg: float,
        end_deg: float,
        step_deg: float = SWEEP_STEP_DEG,
        delay_s: float = SWEEP_DELAY_S,
    ) -> None:
        start_deg = clamp(start_deg, self.min_angle_deg, self.max_angle_deg)
        end_deg = clamp(end_deg, self.min_angle_deg, self.max_angle_deg)

        step = abs(step_deg)
        if end_deg < start_deg:
            step = -step

        angle = start_deg
        while (step > 0 and angle <= end_deg) or (step < 0 and angle >= end_deg):
            self.move_to_angle(angle)
            time.sleep(delay_s)
            angle += step

        self.move_to_angle(end_deg)

    def info(self) -> str:
        return (
            f"addr=0x{I2C_ADDRESS:02X}, channel={self.channel}, freq={self.freq_hz}Hz, "
            f"angle_range={self.min_angle_deg}..{self.max_angle_deg} deg, "
            f"pulse_range={self.min_us}..{self.max_us} us, "
            f"current_angle={self.current_angle_deg}"
        )

    def deinit(self) -> None:
        self.pca.deinit()


def print_help():
    print("""
270° Servo Commands:
  lower                    -> move to 0° (lowered)
  raise                    -> move to 270° (raised)
  angle <deg>              -> move to any angle from 0..270
  center                   -> move to 135°
  range <min_us> <max_us>  -> set mapping pulse endpoints
  pulse <us>               -> directly set a pulse width in microseconds
  sweep <start> <end>      -> sweep using angles in degrees
  info                     -> show current settings
  help                     -> show this help
  quit/exit                -> exit

Examples:
  lower
  raise
  angle 90
  range 600 2400
  sweep 0 270
""")


def main():
    servo = Servo270(start_angle_deg=0.0)
    print("270° servo controller started. Servo set to 0° (lowered). Type 'help'.")

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
                print(servo.info())

            elif cmd == "lower":
                pulse = servo.lower()
                print(f"Moved to 0° (lowered) -> {pulse:.0f} us")

            elif cmd == "raise":
                pulse = servo.raise_up()
                print(f"Moved to 270° (raised) -> {pulse:.0f} us")

            elif cmd == "center":
                pulse = servo.center()
                print(f"Moved to 135° -> {pulse:.0f} us")

            elif cmd == "angle":
                if len(parts) < 2:
                    print("Usage: angle <deg>")
                    continue
                deg = float(parts[1])
                pulse = servo.move_to_angle(deg)
                print(f"Commanded {deg:.1f}° -> {pulse:.0f} us")

            elif cmd == "range":
                if len(parts) < 3:
                    print("Usage: range <min_us> <max_us>")
                    continue
                servo.set_range(float(parts[1]), float(parts[2]))
                print(f"Set range: min_us={servo.min_us}, max_us={servo.max_us}")

            elif cmd == "pulse":
                if len(parts) < 2:
                    print("Usage: pulse <us>")
                    continue
                us = float(parts[1])
                servo.set_pulse_us(us)
                print(f"Pulse set to {us:.0f} us")

            elif cmd == "sweep":
                if len(parts) < 3:
                    print("Usage: sweep <start_deg> <end_deg>")
                    continue
                start_deg = float(parts[1])
                end_deg = float(parts[2])
                print("Sweeping... stop if it binds.")
                servo.sweep_angles(start_deg, end_deg)
                print("Sweep complete.")

            else:
                print("Unknown command. Type 'help'.")

    except KeyboardInterrupt:
        print("\nExiting (Ctrl+C).")
    finally:
        servo.deinit()


if __name__ == "__main__":
    main()