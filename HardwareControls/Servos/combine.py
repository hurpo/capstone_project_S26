#!/usr/bin/env python3
from __future__ import annotations

import os
import sys

SERVOS_DIR = os.path.abspath(os.path.expanduser("~/Documents/CAPSTONE_PROJECT_S26/HardwareControls/Servos"))
if SERVOS_DIR not in sys.path:
    sys.path.insert(0, SERVOS_DIR)

import time

from servo_common import CONFIG_PATH, ServoAppBase, clamp, us_to_duty_16bit


class _ContinuousChannel:
    def __init__(self, parent: "DualContinuousServos", name: str, channel: int, stop_us: int):
        self.parent = parent
        self.name = name
        self.channel = int(channel)
        self.out = parent._pca.channels[self.channel]
        self.stop_us = int(stop_us)
        self.range_us = int(parent.range_us)

    def set_pulse_us(self, pulse_us: float) -> None:
        pulse_us = clamp(pulse_us, self.parent.min_us, self.parent.max_us)
        self.out.duty_cycle = us_to_duty_16bit(pulse_us, self.parent.freq_hz)

    def stop(self) -> None:
        self.set_pulse_us(self.stop_us)

    def off(self) -> None:
        self.out.duty_cycle = 0

    def set_center(self, stop_us: float) -> None:
        self.stop_us = int(clamp(stop_us, self.parent.min_us, self.parent.max_us))
        self.parent.servo_cfg[self.name]["stop_us"] = self.stop_us
        self.stop()

    def set_range(self, range_us: float) -> None:
        self.range_us = int(max(0, range_us))

    def forward(self, speed: float = 1.0) -> None:
        speed = clamp(speed, 0.0, 1.0)
        self.set_pulse_us(self.stop_us + speed * self.range_us)

    def backward(self, speed: float = 1.0) -> None:
        speed = clamp(speed, 0.0, 1.0)
        self.set_pulse_us(self.stop_us - speed * self.range_us)


class DualContinuousServos(ServoAppBase):
    def __init__(self):
        super().__init__("combine")
        self.min_us = int(self.board_cfg.get("continuous_min_us", 1000))
        self.max_us = int(self.board_cfg.get("continuous_max_us", 2000))
        self.range_us = int(self.servo_cfg.get("range_us", self.board_cfg.get("continuous_range_us", 400)))
        self.servo_ch_a = int(self.servo_cfg["servo_a"]["channel"])
        self.servo_ch_b = int(self.servo_cfg["servo_b"]["channel"])
        self.a = _ContinuousChannel(self, "servo_a", self.servo_ch_a, int(self.servo_cfg["servo_a"]["stop_us"]))
        self.b = _ContinuousChannel(self, "servo_b", self.servo_ch_b, int(self.servo_cfg["servo_b"]["stop_us"]))
        self.a_forward_b_backward = not bool(self.servo_cfg.get("invert_pair", False))
        startup_behavior = self.servo_cfg.get("startup_behavior", "stop")
        if startup_behavior == "off":
            self.off_all()
        else:
            self.stop_all()
        time.sleep(self.startup_stop_delay_s)

    def stop_all(self) -> None:
        self.a.stop()
        self.b.stop()

    def off_all(self) -> None:
        self.a.off()
        self.b.off()

    def swap(self) -> None:
        self.a_forward_b_backward = not self.a_forward_b_backward
        self.servo_cfg["invert_pair"] = not self.a_forward_b_backward

    def run_opposite_full(self) -> None:
        if self.a_forward_b_backward:
            self.a.forward(1.0)
            self.b.backward(1.0)
        else:
            self.a.backward(1.0)
            self.b.forward(1.0)

    def set_range(self, range_us: float) -> None:
        self.range_us = int(max(0, range_us))
        self.servo_cfg["range_us"] = self.range_us
        self.a.set_range(self.range_us)
        self.b.set_range(self.range_us)

    def pulse_a(self, pulse_us: float) -> None:
        self.a.set_pulse_us(pulse_us)

    def pulse_b(self, pulse_us: float) -> None:
        self.b.set_pulse_us(pulse_us)

    def status_string(self) -> str:
        mapping = "A forward, B backward" if self.a_forward_b_backward else "A backward, B forward"
        return (
            f"Config file: {CONFIG_PATH}\n"
            f"Servo key: combine\n"
            f"I2C address: 0x{self.i2c_address:02X}\n"
            f"PWM frequency: {self.freq_hz} Hz\n"
            f"Min pulse: {self.min_us} us\n"
            f"Max pulse: {self.max_us} us\n"
            f"Range: ±{self.range_us} us\n"
            f"Servo A: channel={self.servo_ch_a}, stop_us={self.a.stop_us}\n"
            f"Servo B: channel={self.servo_ch_b}, stop_us={self.b.stop_us}\n"
            f"Direction mapping: {mapping}"
        )

    def deinit(self) -> None:
        if self._closed:
            return
        try:
            self.stop_all()
            time.sleep(self.shutdown_stop_delay_s)
        finally:
            super().deinit()


def parse_speed(parts, default: float = 1.0) -> float:
    if len(parts) < 2:
        return default
    return clamp(float(parts[1]), 0.0, 1.0)


def print_help(servos: DualContinuousServos) -> None:
    print(f"""
Servo A channel={servos.servo_ch_a}, Servo B channel={servos.servo_ch_b}
Config file: {CONFIG_PATH}

Commands:
  run
  stop
  off
  centera <us>
  centerb <us>
  centerboth <us_a> <us_b>
  range <us>
  swap
  pulsea <us>
  pulseb <us>
  forwarda [speed]
  backwarda [speed]
  forwardb [speed]
  backwardb [speed]
  status
  save
  help
  quit / exit
""")


def main() -> None:
    servos = DualContinuousServos()
    servos._install_cleanup()
    print("Dual continuous servo control using PCA9685.")
    print(servos.status_string())
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
                print_help(servos)
            elif cmd == "status":
                print(servos.status_string())
            elif cmd == "run":
                servos.run_opposite_full()
                print("Running both combine servos.")
            elif cmd == "stop":
                servos.stop_all()
                print("Stopped using calibrated neutral pulses.")
            elif cmd == "off":
                servos.off_all()
                print("Both outputs set to duty_cycle=0.")
            elif cmd == "swap":
                servos.swap()
                print("Swapped direction mapping.")
            elif cmd == "centera":
                servos.a.set_center(float(parts[1]))
                print(f"Servo A neutral set to {servos.a.stop_us} us")
            elif cmd == "centerb":
                servos.b.set_center(float(parts[1]))
                print(f"Servo B neutral set to {servos.b.stop_us} us")
            elif cmd == "centerboth":
                servos.a.set_center(float(parts[1]))
                servos.b.set_center(float(parts[2]))
                print("Updated both neutral values.")
            elif cmd == "range":
                servos.set_range(float(parts[1]))
                print(f"Range set to ±{servos.range_us} us")
            elif cmd == "pulsea":
                servos.pulse_a(float(parts[1]))
                print(f"Set servo A pulse on channel {servos.servo_ch_a} to {parts[1]} us")
            elif cmd == "pulseb":
                servos.pulse_b(float(parts[1]))
                print(f"Set servo B pulse on channel {servos.servo_ch_b} to {parts[1]} us")
            elif cmd == "forwarda":
                servos.a.forward(parse_speed(parts))
                print("Servo A forward command sent.")
            elif cmd == "backwarda":
                servos.a.backward(parse_speed(parts))
                print("Servo A backward command sent.")
            elif cmd == "forwardb":
                servos.b.forward(parse_speed(parts))
                print("Servo B forward command sent.")
            elif cmd == "backwardb":
                servos.b.backward(parse_speed(parts))
                print("Servo B backward command sent.")
            elif cmd == "save":
                servos.save()
                print(f"Saved configuration to {CONFIG_PATH}")
            else:
                print("Unknown command. Type 'help'.")
        except Exception as exc:
            print(f"Error: {exc}")

    servos.deinit()


if __name__ == "__main__":
    main()
