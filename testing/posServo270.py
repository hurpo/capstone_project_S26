#!/usr/bin/env python3
from __future__ import annotations

import argparse
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

I2C_ADDRESS = 0x40
CHANNEL = 1
PWM_FREQUENCY_HZ = 50

MIN_US_DEFAULT = 500
MAX_US_DEFAULT = 2500
MAX_ANGLE_DEG = 270.0

SWEEP_STEP_US = 10
SWEEP_DELAY_S = 0.02
DEFAULT_START_ANGLE = 135.0


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
    def __init__(
        self,
        address: int = I2C_ADDRESS,
        channel: int = CHANNEL,
        freq_hz: int = PWM_FREQUENCY_HZ,
        initial_angle_deg: float = DEFAULT_START_ANGLE,
    ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq_hz

        self.freq_hz = freq_hz
        self.channel = channel
        self.out = self.pca.channels[channel]

        self.min_us = float(MIN_US_DEFAULT)
        self.max_us = float(MAX_US_DEFAULT)
        self.max_angle_deg = float(MAX_ANGLE_DEG)

        # Track the current angle without commanding motion on startup.
        self.current_angle_deg = clamp(float(initial_angle_deg), 0.0, self.max_angle_deg)

    def set_pulse_us(self, pulse_us: float) -> None:
        period_us = 1_000_000.0 / self.freq_hz
        pulse_us = clamp(pulse_us, 0.0, period_us)
        self.out.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def set_angle(self, angle_deg: float) -> float:
        angle_deg = clamp(angle_deg, 0.0, self.max_angle_deg)
        pulse = angle_to_us(angle_deg, self.min_us, self.max_us, self.max_angle_deg)
        self.set_pulse_us(pulse)
        self.current_angle_deg = angle_deg
        return pulse

    def add_angle(self, delta_deg: float) -> float:
        return self.set_angle(self.current_angle_deg + float(delta_deg))

    def center(self) -> float:
        return self.set_angle(self.max_angle_deg / 2.0)

    def sweep(self, start_us: float, end_us: float, step_us: int = SWEEP_STEP_US, delay_s: float = SWEEP_DELAY_S) -> None:
        start_us = int(start_us)
        end_us = int(end_us)
        step = abs(int(step_us)) if end_us >= start_us else -abs(int(step_us))

        us = start_us
        while (step > 0 and us <= end_us) or (step < 0 and us >= end_us):
            self.set_pulse_us(us)
            time.sleep(delay_s)
            us += step

    def release(self) -> None:
        self.out.duty_cycle = 0

    def deinit(self) -> None:
        self.pca.deinit()


def print_help() -> None:
    print(
        '''
270° Servo Commands:
  angle <deg>              -> command 0..270 degrees
  add <deg>                -> add delta degrees to current tracked angle
  center                   -> go to 135 degrees
  range <min_us> <max_us>  -> set mapping pulse endpoints
  pulse <us>               -> set a specific pulse width in microseconds
  sweep <min_us> <max_us>  -> sweep pulses slowly
  release                  -> stop PWM output
  info                     -> show current settings and tracked angle
  help                     -> show this help
  quit/exit                -> exit

Notes:
- Startup does not command any movement.
- 'current angle' is the angle last commanded by this script.
- If the servo was moved by some other script or by hand, use 'angle <deg>' first.
'''
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control a 270 degree PCA9685 servo without moving it on startup.")
    parser.add_argument("--channel", type=int, default=CHANNEL, help="PCA9685 channel")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=I2C_ADDRESS, help="I2C address, e.g. 0x40")
    parser.add_argument("--freq", type=int, default=PWM_FREQUENCY_HZ, help="PWM frequency in Hz")
    parser.add_argument(
        "--start-angle",
        type=float,
        default=DEFAULT_START_ANGLE,
        help="Initial tracked angle only; no startup motion is sent",
    )

    subparsers = parser.add_subparsers(dest="command")

    p_angle = subparsers.add_parser("angle", help="Command an absolute angle")
    p_angle.add_argument("deg", type=float)

    p_add = subparsers.add_parser("add", help="Add a delta angle to the current tracked angle")
    p_add.add_argument("--delta", type=float, required=True)

    subparsers.add_parser("center", help="Move to 135 degrees")

    p_range = subparsers.add_parser("range", help="Change pulse endpoint calibration")
    p_range.add_argument("min_us", type=float)
    p_range.add_argument("max_us", type=float)

    p_pulse = subparsers.add_parser("pulse", help="Set pulse width in microseconds")
    p_pulse.add_argument("us", type=float)

    p_sweep = subparsers.add_parser("sweep", help="Sweep from one pulse width to another")
    p_sweep.add_argument("min_us", type=float)
    p_sweep.add_argument("max_us", type=float)

    subparsers.add_parser("info", help="Show current settings")
    subparsers.add_parser("release", help="Disable PWM output")
    subparsers.add_parser("interactive", help="Start interactive shell")

    return parser


def interactive_shell(servo: Servo270) -> None:
    print(f"PCA9685 270° servo controller (channel {servo.channel}). Type 'help'.")
    print("Startup sent no motion command.")

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
            print(f"tracked_angle={servo.current_angle_deg:.1f}°")
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
            print(f"Commanded {servo.current_angle_deg:.1f}° -> {pulse:.0f} us")
        elif cmd == "add":
            if len(parts) < 2:
                print("Usage: add <deg>")
                continue
            delta = float(parts[1])
            pulse = servo.add_angle(delta)
            print(f"Added {delta:.1f}° -> now {servo.current_angle_deg:.1f}° -> {pulse:.0f} us")
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
        elif cmd == "release":
            servo.release()
            print("PWM released.")
        else:
            print("Unknown command. Type 'help'.")


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    servo = Servo270(
        address=args.address,
        channel=args.channel,
        freq_hz=args.freq,
        initial_angle_deg=args.start_angle,
    )

    try:
        if args.command is None or args.command == "interactive":
            interactive_shell(servo)
        elif args.command == "angle":
            pulse = servo.set_angle(args.deg)
            print(f"Commanded {servo.current_angle_deg:.1f}° -> {pulse:.0f} us")
        elif args.command == "add":
            pulse = servo.add_angle(args.delta)
            print(f"Added {args.delta:.1f}° -> now {servo.current_angle_deg:.1f}° -> {pulse:.0f} us")
        elif args.command == "center":
            pulse = servo.center()
            print(f"Centered at 135° -> {pulse:.0f} us")
        elif args.command == "range":
            servo.min_us = float(args.min_us)
            servo.max_us = float(args.max_us)
            print(f"Set range: min_us={servo.min_us}, max_us={servo.max_us}")
        elif args.command == "pulse":
            servo.set_pulse_us(args.us)
            print(f"Pulse set to {int(args.us)} us")
        elif args.command == "sweep":
            print("Sweeping... stop if it binds.")
            servo.sweep(args.min_us, args.max_us)
            servo.sweep(args.max_us, args.min_us)
            print("Sweep complete.")
        elif args.command == "info":
            print(f"addr=0x{I2C_ADDRESS:02X}, channel={servo.channel}, freq={servo.freq_hz}Hz")
            print(f"max_angle={servo.max_angle_deg}°  min_us={servo.min_us}  max_us={servo.max_us}")
            print(f"tracked_angle={servo.current_angle_deg:.1f}°")
        elif args.command == "release":
            servo.release()
            print("PWM released.")
    except KeyboardInterrupt:
        print("\nExiting (Ctrl+C).")
    finally:
        servo.deinit()


if __name__ == "__main__":
    main()
