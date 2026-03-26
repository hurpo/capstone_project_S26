from __future__ import annotations

import board
import busio
from adafruit_pca9685 import PCA9685


I2C_ADDRESS = 0x40
CHANNEL = 15
FREQ_HZ = 50
MIN_US = 500.0
MAX_US = 2500.0
MAX_ANGLE_DEG = 180.0
DEFAULT_DEG = 90.0


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def us_to_duty_16bit(pulse_us: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / freq_hz
    duty_fraction = clamp(pulse_us / period_us, 0.0, 1.0)
    return int(duty_fraction * 65535.0)


def angle_to_us(angle_deg: float, min_us: float, max_us: float, max_angle_deg: float) -> float:
    angle_deg = clamp(angle_deg, 0.0, max_angle_deg)
    return min_us + (angle_deg / max_angle_deg) * (max_us - min_us)


class CameraServo:
    """180 degree positional micro servo for the camera."""

    def __init__(self, address: int = I2C_ADDRESS, channel: int = CHANNEL, freq_hz: int = FREQ_HZ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq_hz

        self.freq_hz = freq_hz
        self.ch = self.pca.channels[channel]
        self.min_us = float(MIN_US)
        self.max_us = float(MAX_US)
        self.max_angle_deg = float(MAX_ANGLE_DEG)
        self.last_commanded_deg = DEFAULT_DEG

        self.set_angle(DEFAULT_DEG)

    def set_pulse_us(self, pulse_us: float) -> None:
        self.ch.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def set_angle(self, angle_deg: float) -> float:
        angle_deg = clamp(angle_deg, 0.0, self.max_angle_deg)
        pulse = angle_to_us(angle_deg, self.min_us, self.max_us, self.max_angle_deg)
        self.set_pulse_us(pulse)
        self.last_commanded_deg = angle_deg
        return pulse

    def look_forward(self) -> float:
        return self.set_angle(90.0)

    def look_down(self) -> float:
        return self.set_angle(120.0)

    def look_up(self) -> float:
        return self.set_angle(60.0)

    def deinit(self) -> None:
        self.pca.deinit()
