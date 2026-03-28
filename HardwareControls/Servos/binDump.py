from __future__ import annotations

import argparse
import time

import board
import busio
from adafruit_pca9685 import PCA9685


I2C_ADDRESS = 0x40
CHANNEL = 7
FREQ_HZ = 50
MIN_US = 500.0
MAX_US = 2500.0
MAX_ANGLE_DEG = 270.0
CLOSED_DEG = 220.0
OPEN_DEG = 60.0


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def us_to_duty_16bit(pulse_us: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / freq_hz
    duty_fraction = clamp(pulse_us / period_us, 0.0, 1.0)
    return int(duty_fraction * 65535.0)


def angle_to_us(angle_deg: float, min_us: float, max_us: float, max_angle_deg: float) -> float:
    angle_deg = clamp(angle_deg, 0.0, max_angle_deg)
    return min_us + (angle_deg / max_angle_deg) * (max_us - min_us)


class BinDumpServo:
    """270 degree positional servo for the bin dump mechanism."""

    def __init__(
        self,
        address: int = I2C_ADDRESS,
        channel: int = CHANNEL,
        freq_hz: int = FREQ_HZ,
        auto_initialize: bool = True,
    ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq_hz

        self.address = address
        self.channel = channel
        self.freq_hz = freq_hz
        self.ch = self.pca.channels[channel]
        self.min_us = float(MIN_US)
        self.max_us = float(MAX_US)
        self.max_angle_deg = float(MAX_ANGLE_DEG)
        self.last_commanded_deg = CLOSED_DEG
        self.is_open = False

        if auto_initialize:
            self.close()

    def set_pulse_us(self, pulse_us: float) -> None:
        self.ch.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def set_angle(self, angle_deg: float) -> float:
        angle_deg = clamp(angle_deg, 0.0, self.max_angle_deg)
        pulse = angle_to_us(angle_deg, self.min_us, self.max_us, self.max_angle_deg)
        self.set_pulse_us(pulse)
        self.last_commanded_deg = angle_deg
        self.is_open = abs(angle_deg - OPEN_DEG) < abs(angle_deg - CLOSED_DEG)
        return pulse

    def open(self) -> float:
        self.is_open = True
        return self.set_angle(OPEN_DEG)

    def close(self) -> float:
        self.is_open = False
        return self.set_angle(CLOSED_DEG)

    def toggle(self) -> float:
        if self.is_open:
            return self.close()
        return self.open()

    def hold(self, seconds: float) -> None:
        time.sleep(max(0.0, seconds))

    def release(self) -> None:
        self.ch.duty_cycle = 0

    def deinit(self) -> None:
        self.pca.deinit()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control the bin dump servo directly.")
    parser.add_argument(
        "command",
        nargs="?",
        default="interactive",
        choices=["open", "close", "toggle", "angle", "interactive"],
        help="Servo action to perform",
    )
    parser.add_argument(
        "--angle",
        type=float,
        default=None,
        help="Angle in degrees for the angle command",
    )
    parser.add_argument("--channel", type=int, default=CHANNEL, help="PCA9685 channel")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=I2C_ADDRESS, help="I2C address, e.g. 0x40")
    parser.add_argument("--freq", type=int, default=FREQ_HZ, help="PWM frequency in Hz")
    parser.add_argument(
        "--hold",
        type=float,
        default=0.75,
        help="Time in seconds to hold the command before optionally releasing PWM",
    )
    parser.add_argument(
        "--no-release",
        action="store_true",
        help="Keep PWM applied after the command instead of setting duty cycle to 0",
    )
    return parser


def run_command(servo: BinDumpServo, command: str, angle: float | None = None) -> None:
    if command == "open":
        servo.open()
        print(f"Bin dump opened to {OPEN_DEG:.1f} degrees.")
    elif command == "close":
        servo.close()
        print(f"Bin dump closed to {CLOSED_DEG:.1f} degrees.")
    elif command == "toggle":
        servo.toggle()
        state = "open" if servo.is_open else "closed"
        print(f"Bin dump toggled to {state} at {servo.last_commanded_deg:.1f} degrees.")
    elif command == "angle":
        if angle is None:
            raise ValueError("The angle command requires --angle.")
        servo.set_angle(angle)
        print(f"Bin dump moved to {servo.last_commanded_deg:.1f} degrees.")
    else:
        raise ValueError(f"Unsupported command: {command}")


def interactive_loop(servo: BinDumpServo) -> None:
    print("Interactive bin dump control")
    print("Commands: open, close, toggle, angle <deg>, status, release, quit")

    while True:
        try:
            raw = input("binDump> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        try:
            if cmd in ("quit", "exit", "q"):
                break
            elif cmd == "open":
                servo.open()
                print(f"Opened to {servo.last_commanded_deg:.1f} degrees.")
            elif cmd == "close":
                servo.close()
                print(f"Closed to {servo.last_commanded_deg:.1f} degrees.")
            elif cmd == "toggle":
                servo.toggle()
                state = "open" if servo.is_open else "closed"
                print(f"Toggled to {state} at {servo.last_commanded_deg:.1f} degrees.")
            elif cmd == "angle":
                if len(parts) != 2:
                    print("Usage: angle <degrees>")
                    continue
                servo.set_angle(float(parts[1]))
                print(f"Moved to {servo.last_commanded_deg:.1f} degrees.")
            elif cmd == "status":
                state = "open" if servo.is_open else "closed"
                print(f"State: {state}, angle: {servo.last_commanded_deg:.1f} degrees.")
            elif cmd == "release":
                servo.release()
                print("PWM released.")
            else:
                print("Unknown command.")
        except Exception as exc:
            print(f"Error: {exc}")


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    servo = BinDumpServo(
        address=args.address,
        channel=args.channel,
        freq_hz=args.freq,
        auto_initialize=False,
    )

    try:
        if args.command == "interactive":
            servo.close()
            interactive_loop(servo)
        else:
            run_command(servo, args.command, args.angle)
            servo.hold(args.hold)
            if not args.no_release:
                servo.release()
    finally:
        servo.deinit()


if __name__ == "__main__":
    main()
