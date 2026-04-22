#!/usr/bin/env python3
from __future__ import annotations

from servo_common import CONFIG_PATH, PositionalServoBase, clamp


class CameraServo(PositionalServoBase):
    def __init__(self):
        super().__init__("camera")

    def forward(self) -> float:
        return self.move_to_named("forward")

    def down(self) -> float:
        return self.move_to_named("down")

    def up(self) -> float:
        return self.move_to_named("up")


def print_help(servo: CameraServo) -> None:
    print(f"""
Config file: {CONFIG_PATH}
Servo key: camera

Commands:
  positions                -> list named positions from JSON
  forward
  down
  up
  angle <deg>              -> move to an absolute angle
  pulse <us>               -> set a direct pulse width
  range <min_us> <max_us>  -> set pulse endpoint calibration
  setpos <name> <deg>      -> change a named position in memory
  sweep <start> <end> [step] [delay]
  release                  -> set duty_cycle=0
  status
  save
  help
  quit / exit
""")


def main() -> None:
    servo = CameraServo()
    servo._install_cleanup()
    print(servo.status_string())
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
                print_help(servo)
            elif cmd == "status":
                print(servo.status_string())
            elif cmd == "positions":
                for name, angle in servo.positions.items():
                    print(f"{name} = {angle} deg")
            elif cmd in servo.positions:
                pulse = servo.move_to_named(cmd)
                print(f"Moved to '{cmd}' -> {pulse:.0f} us")
            elif cmd == "angle":
                pulse = servo.set_angle(float(parts[1]))
                print(f"Moved to {servo.current_angle_deg:.1f} deg -> {pulse:.0f} us")
            elif cmd == "pulse":
                servo.set_pulse_us(float(parts[1]))
                print(f"Pulse set to {float(parts[1]):.0f} us")
            elif cmd == "range":
                servo.set_range(float(parts[1]), float(parts[2]))
                print(f"Pulse range set to {servo.min_us}..{servo.max_us} us")
            elif cmd == "setpos":
                servo.set_named_position(parts[1], float(parts[2]))
                print(f"Updated named position '{parts[1]}' to {servo.positions[parts[1]]} deg")
            elif cmd == "sweep":
                step = float(parts[3]) if len(parts) >= 4 else 2.0
                delay = float(parts[4]) if len(parts) >= 5 else 0.02
                servo.sweep(float(parts[1]), float(parts[2]), step, delay)
                print("Sweep complete.")
            elif cmd == "release":
                servo.release()
                print("PWM released.")
            elif cmd == "save":
                servo.save()
                print(f"Saved configuration to {CONFIG_PATH}")
            else:
                print("Unknown command. Type 'help'.")
        except Exception as exc:
            print(f"Error: {exc}")

    servo.deinit()


if __name__ == "__main__":
    main()
