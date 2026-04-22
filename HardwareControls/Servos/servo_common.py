from __future__ import annotations

import atexit
import json
import signal
import time
from pathlib import Path
from typing import Any

import board
import busio
from adafruit_pca9685 import PCA9685


CONFIG_PATH = Path(__file__).resolve().parent / "servo_config.json"


DEFAULT_CONFIG: dict[str, Any] = {
    "board": {
        "i2c_address": 64,
        "pwm_frequency_hz": 50,
        "continuous_min_us": 1000,
        "continuous_max_us": 2000,
        "continuous_range_us": 400,
        "positional_min_us": 500,
        "positional_max_us": 2500,
        "startup_stop_delay_s": 0.5,
        "shutdown_stop_delay_s": 0.5,
    },
    "combine": {
        "type": "continuous_pair",
        "servo_a": {"channel": 3, "stop_us": 1700},
        "servo_b": {"channel": 4, "stop_us": 1700},
        "invert_pair": False,
        "startup_behavior": "stop",
    },
    "conveyor": {
        "type": "continuous",
        "channel": 8,
        "stop_us": 1525,
        "range_us": 400,
        "startup_behavior": "stop",
    },
    "camera": {
        "type": "positional",
        "channel": 15,
        "servo_degrees": 180,
        "min_us": 500,
        "max_us": 2500,
        "startup_position": "forward",
        "positions": {
            "forward": 90.0,
            "down": 120.0,
            "up": 60.0,
        },
    },
    "chute": {
        "type": "positional",
        "channel": 2,
        "servo_degrees": 180,
        "min_us": 500,
        "max_us": 2500,
        "startup_position": "close",
        "positions": {
            "close": 80.0,
            "open": 15.0,
        },
    },
    "rackpinion": {
        "type": "positional",
        "channel": 5,
        "servo_degrees": 270,
        "min_us": 500,
        "max_us": 2500,
        "startup_position": "lower",
        "positions": {
            "lower": 0.0,
            "raise": 270.0,
            "center": 135.0,
        },
    },
    "falseFloor": {
        "type": "positional",
        "channel": 6,
        "servo_degrees": 270,
        "min_us": 500,
        "max_us": 2500,
        "startup_position": "close",
        "positions": {
            "close": 210.0,
            "open": 135.0,
            "center": 210.0,
        },
    },
    "clawPincher": {
        "type": "positional",
        "channel": 8,
        "servo_degrees": 270,
        "min_us": 500,
        "max_us": 2500,
        "startup_position": "close",
        "positions": {
            "close": 108.0,
            "open": 80.0,
            "latched": 20.0,
        },
    },
    "clawBase": {
        "type": "positional",
        "channel": 1,
        "servo_degrees": 270,
        "min_us": 500,
        "max_us": 2500,
        "startup_position": "initialize",
        "positions": {
            "initialize": 200.0,
            "extend": 90.0,
            "retract": 270.0,
        },
    },
    "binDump": {
        "type": "positional",
        "channel": 7,
        "servo_degrees": 270,
        "min_us": 500,
        "max_us": 2500,
        "startup_position": "close",
        "positions": {
            "close": 220.0,
            "open": 60.0,
        },
    },
}


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def us_to_duty_16bit(microseconds: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / freq_hz
    duty_fraction = clamp(microseconds / period_us, 0.0, 1.0)
    return int(duty_fraction * 65535.0)


def angle_to_us(angle_deg: float, min_us: float, max_us: float, max_angle_deg: float) -> float:
    angle_deg = clamp(angle_deg, 0.0, max_angle_deg)
    return min_us + (angle_deg / max_angle_deg) * (max_us - min_us)


def load_config() -> dict[str, Any]:
    if not CONFIG_PATH.exists():
        save_config(DEFAULT_CONFIG)
        return json.loads(json.dumps(DEFAULT_CONFIG))
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    return cfg


def save_config(cfg: dict[str, Any]) -> None:
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)


class ServoAppBase:
    def __init__(self, config_key: str):
        self.config_key = config_key
        self.config = load_config()
        self.board_cfg = self.config["board"]
        self.servo_cfg = self.config[config_key]
        self.i2c_address = int(self.board_cfg["i2c_address"])
        self.freq_hz = int(self.board_cfg["pwm_frequency_hz"])
        self.startup_stop_delay_s = float(self.board_cfg.get("startup_stop_delay_s", 0.5))
        self.shutdown_stop_delay_s = float(self.board_cfg.get("shutdown_stop_delay_s", 0.5))
        self._closed = False
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(self._i2c, address=self.i2c_address)
        self._pca.frequency = self.freq_hz

    def save(self) -> None:
        save_config(self.config)

    def release_channel(self, channel: int) -> None:
        self._pca.channels[channel].duty_cycle = 0

    def _install_cleanup(self) -> None:
        holder = self

        def cleanup() -> None:
            holder.deinit()

        def signal_handler(signum, frame):
            cleanup()
            raise SystemExit(0)

        atexit.register(cleanup)
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

    def deinit(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._pca.deinit()


class ContinuousServoBase(ServoAppBase):
    def __init__(self, config_key: str):
        super().__init__(config_key)
        self.channel = int(self.servo_cfg["channel"])
        self.out = self._pca.channels[self.channel]
        self.min_us = int(self.board_cfg.get("continuous_min_us", 1000))
        self.max_us = int(self.board_cfg.get("continuous_max_us", 2000))
        self.stop_us = int(self.servo_cfg["stop_us"])
        self.range_us = int(self.servo_cfg.get("range_us", self.board_cfg.get("continuous_range_us", 400)))
        self.direction = 1
        self.speed = 1.0
        startup_behavior = self.servo_cfg.get("startup_behavior", "stop")
        if startup_behavior == "off":
            self.off()
        else:
            self.stop()
        time.sleep(self.startup_stop_delay_s)

    def set_pulse_us(self, pulse_us: float) -> None:
        pulse_us = clamp(pulse_us, self.min_us, self.max_us)
        self.out.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def stop(self) -> None:
        self.set_pulse_us(self.stop_us)

    def off(self) -> None:
        self.out.duty_cycle = 0

    def set_center(self, stop_us: float) -> None:
        self.stop_us = int(clamp(stop_us, self.min_us, self.max_us))
        self.servo_cfg["stop_us"] = self.stop_us
        self.stop()

    def set_range(self, range_us: float) -> None:
        self.range_us = int(max(0, range_us))
        self.servo_cfg["range_us"] = self.range_us

    def forward(self, speed: float = 1.0) -> None:
        self.direction = 1
        self.speed = clamp(speed, 0.0, 1.0)
        self.set_pulse_us(self.stop_us + self.speed * self.range_us)

    def reverse(self, speed: float = 1.0) -> None:
        self.direction = -1
        self.speed = clamp(speed, 0.0, 1.0)
        self.set_pulse_us(self.stop_us - self.speed * self.range_us)

    def set_speed(self, speed: float) -> None:
        if self.direction >= 0:
            self.forward(speed)
        else:
            self.reverse(speed)

    def set_direction(self, forward: bool) -> None:
        self.direction = 1 if forward else -1
        self.set_speed(self.speed)

    def status_string(self) -> str:
        direction_str = "forward" if self.direction >= 0 else "reverse"
        return (
            f"Config file: {CONFIG_PATH}\n"
            f"Servo key: {self.config_key}\n"
            f"I2C address: 0x{self.i2c_address:02X}\n"
            f"Channel: {self.channel}\n"
            f"PWM frequency: {self.freq_hz} Hz\n"
            f"Neutral stop_us: {self.stop_us} us\n"
            f"Range: ±{self.range_us} us\n"
            f"Direction: {direction_str}\n"
            f"Speed: {self.speed:.2f}"
        )

    def deinit(self) -> None:
        if self._closed:
            return
        try:
            self.stop()
            time.sleep(self.shutdown_stop_delay_s)
        finally:
            super().deinit()


class PositionalServoBase(ServoAppBase):
    def __init__(self, config_key: str):
        super().__init__(config_key)
        self.channel = int(self.servo_cfg["channel"])
        self.out = self._pca.channels[self.channel]
        self.min_us = float(self.servo_cfg.get("min_us", self.board_cfg.get("positional_min_us", 500)))
        self.max_us = float(self.servo_cfg.get("max_us", self.board_cfg.get("positional_max_us", 2500)))
        self.max_angle_deg = float(self.servo_cfg.get("servo_degrees", 180))
        self.positions = dict(self.servo_cfg.get("positions", {}))
        self.current_angle_deg: float | None = None
        startup_position = self.servo_cfg.get("startup_position")
        if startup_position and startup_position in self.positions:
            self.move_to_named(startup_position)

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

    def move_to_named(self, name: str) -> float:
        if name not in self.positions:
            raise ValueError(f"Unknown named position: {name}")
        return self.set_angle(float(self.positions[name]))

    def set_range(self, min_us: float, max_us: float) -> None:
        self.min_us = float(min_us)
        self.max_us = float(max_us)
        self.servo_cfg["min_us"] = self.min_us
        self.servo_cfg["max_us"] = self.max_us

    def set_named_position(self, name: str, angle_deg: float) -> None:
        angle_deg = float(clamp(angle_deg, 0.0, self.max_angle_deg))
        self.positions[name] = angle_deg
        self.servo_cfg.setdefault("positions", {})[name] = angle_deg

    def sweep(self, start_deg: float, end_deg: float, step_deg: float = 2.0, delay_s: float = 0.02) -> None:
        start_deg = clamp(start_deg, 0.0, self.max_angle_deg)
        end_deg = clamp(end_deg, 0.0, self.max_angle_deg)
        step = abs(step_deg)
        if end_deg < start_deg:
            step = -step
        angle = start_deg
        while (step > 0 and angle <= end_deg) or (step < 0 and angle >= end_deg):
            self.set_angle(angle)
            time.sleep(delay_s)
            angle += step
        self.set_angle(end_deg)

    def release(self) -> None:
        self.out.duty_cycle = 0

    def status_string(self) -> str:
        named = ", ".join(f"{k}={v}" for k, v in self.positions.items())
        return (
            f"Config file: {CONFIG_PATH}\n"
            f"Servo key: {self.config_key}\n"
            f"I2C address: 0x{self.i2c_address:02X}\n"
            f"Channel: {self.channel}\n"
            f"PWM frequency: {self.freq_hz} Hz\n"
            f"Pulse range: {self.min_us}..{self.max_us} us\n"
            f"Max angle: {self.max_angle_deg} deg\n"
            f"Current angle: {self.current_angle_deg}\n"
            f"Named positions: {named}"
        )
