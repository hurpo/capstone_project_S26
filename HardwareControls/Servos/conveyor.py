#!/usr/bin/env python3
from __future__ import annotations

from servo_common import CONFIG_PATH, ContinuousServoBase, clamp


class ConveyorServo(ContinuousServoBase):
    def __init__(self):
        super().__init__("conveyor")

    def run_match_combine(self, reverse: bool = False, speed: float = 1.0) -> None:
        if reverse:
            self.reverse(speed)
        else:
            self.forward(speed)


def parse_speed(parts, default: float = 1.0) -> float:
    if len(parts) < 2:
        return default
    return clamp(float(parts[1]), 0.0, 1.0)


def print_help() -> None:
    print(f"""
Config file: {CONFIG_PATH}
Servo key: conveyor

Commands:
  forward [speed]
  reverse [speed]
  speed <0.0-1.0>
  direction <f|r>
  match [f|r] [speed]
  stop
  center <us>
  range <us>
  pulse <us>
  off
  status
  save
  help
  quit / exit
""")


def main() -> None:
    conveyor = ConveyorServo()
    conveyor._install_cleanup()
    print(conveyor.status_string())
    print("Type 'help' for commands.")

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        try:
            if cmd in ("quit", "exit"):
                break
            elif cmd == "help":
                print_help()
            elif cmd == "status":
                print(conveyor.status_string())
            elif cmd == "forward":
                conveyor.forward(parse_speed(parts))
                print(f"Forward at speed {conveyor.speed:.2f}")
            elif cmd == "reverse":
                conveyor.reverse(parse_speed(parts))
                print(f"Reverse at speed {conveyor.speed:.2f}")
            elif cmd == "speed":
                conveyor.set_speed(float(parts[1]))
                print(f"Speed set to {conveyor.speed:.2f}")
            elif cmd == "direction":
                conveyor.set_direction(parts[1].lower() in ("f", "forward"))
                print("Direction updated.")
            elif cmd == "match":
                reverse = len(parts) >= 2 and parts[1].lower() in ("r", "reverse", "rev")
                speed = clamp(float(parts[2]), 0.0, 1.0) if len(parts) >= 3 else 1.0
                conveyor.run_match_combine(reverse=reverse, speed=speed)
                print("Match command sent.")
            elif cmd == "stop":
                conveyor.stop()
                print("Stopped using calibrated neutral pulse.")
            elif cmd == "center":
                conveyor.set_center(float(parts[1]))
                print(f"Neutral set to {conveyor.stop_us} us")
            elif cmd == "range":
                conveyor.set_range(float(parts[1]))
                print(f"Range set to ±{conveyor.range_us} us")
            elif cmd == "pulse":
                conveyor.set_pulse_us(float(parts[1]))
                print(f"Pulse set to {float(parts[1]):.0f} us")
            elif cmd == "off":
                conveyor.off()
                print("Output set to duty_cycle=0.")
            elif cmd == "save":
                conveyor.save()
                print(f"Saved configuration to {CONFIG_PATH}")
            else:
                print("Unknown command. Type 'help'.")
        except Exception as exc:
            print(f"Error: {exc}")

    conveyor.deinit()


if __name__ == "__main__":
    main()
