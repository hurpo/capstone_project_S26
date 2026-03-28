from __future__ import annotations

import time

import board
import busio
from adafruit_pca9685 import PCA9685


I2C_ADDRESS = 0x40
CHANNEL = 8
PWM_FREQUENCY_HZ = 50

STOP_US = 1525
RANGE_US = 400
MIN_US = 1000
MAX_US = 2000


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def us_to_duty_16bit(microseconds: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / freq_hz
    duty_fraction = clamp(microseconds / period_us, 0.0, 1.0)
    return int(duty_fraction * 65535.0)


class ConveyorServo:
    """
    Continuous servo for the conveyor subsystem.
    Designed to run at the same time and speed as the combine when requested,
    but also supports independent speed and direction changes.
    """

    def __init__(self, address: int = I2C_ADDRESS, channel: int = CHANNEL, freq_hz: int = PWM_FREQUENCY_HZ):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq_hz

        self.freq_hz = freq_hz
        self.channel = channel
        self.out = self.pca.channels[channel]

        self.stop_us = STOP_US
        self.range_us = RANGE_US
        self.direction = 1
        self.speed = 1.0

        self.stop()

    def set_pulse_us(self, pulse_us: float) -> None:
        pulse_us = clamp(pulse_us, MIN_US, MAX_US)
        self.out.duty_cycle = us_to_duty_16bit(pulse_us, self.freq_hz)

    def hard_off(self) -> None:
        self.out.duty_cycle = 0

    def stop(self) -> None:
        self.set_pulse_us(self.stop_us)

    def set_speed(self, speed: float) -> None:
        self.speed = clamp(speed, 0.0, 1.0)
        if self.direction >= 0:
            self.forward(self.speed)
        else:
            self.reverse(self.speed)

    def set_direction(self, forward: bool = True) -> None:
        self.direction = 1 if forward else -1
        self.set_speed(self.speed)

    def forward(self, speed: float = 1.0) -> None:
        print()
        self.direction = 1
        self.speed = clamp(speed, 0.0, 1.0)
        self.set_pulse_us(self.stop_us + self.speed * self.range_us)

    def reverse(self, speed: float = 1.0) -> None:
        self.direction = -1
        self.speed = clamp(speed, 0.0, 1.0)
        self.set_pulse_us(self.stop_us - self.speed * self.range_us)

    def run_match_combine(self, reverse: bool = False, speed: float = 1.0) -> None:
        """
        Match the combine state. If the combine intake is reversed, the conveyor is
        also reversed. Adjust this mapping if your conveyor is physically mounted
        opposite and needs inverted behavior.
        """
        if reverse:
            self.reverse(speed)
        else:
            self.forward(speed)

    def center(self, stop_us: float) -> None:
        self.stop_us = int(stop_us)
        self.stop()

    def deinit(self) -> None:
        try:
            self.stop()
            time.sleep(0.1)
            self.hard_off()
        finally:
            self.pca.deinit()

if __name__ == "__main__":
    s = ConveyorServo()
    print(s)

    s.forward()
    time.sleep(1)
    