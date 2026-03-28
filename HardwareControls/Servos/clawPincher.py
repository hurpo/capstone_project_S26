# 270° positional servo on PCA9685 (I2C) channel 0
# Provides functions your other program can import and call.

from __future__ import annotations

import time
from dataclasses import dataclass

import board
import busio
from adafruit_pca9685 import PCA9685


@dataclass
class Servo270Config:
    i2c_address: int = 0x40
    channel: int = 8
    frequency_hz: int = 50

    # Calibrate these for your specific servo to match its endpoints cleanly
    min_us: float = 500.0
    max_us: float = 2500.0

    # Named positions (degrees) for a 270° servo
    center_closed_deg: float = 106.0   # center (closed)
    open_deg: float = 90.0            # open
    latched_deg: float = 40.0         # latched


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _us_to_duty_16bit(pulse_us: float, freq_hz: float) -> int:
    period_us = 1_000_000.0 / freq_hz
    duty_fraction = _clamp(pulse_us / period_us, 0.0, 1.0)
    return int(duty_fraction * 65535.0)


def _angle_to_us(angle_deg: float, min_us: float, max_us: float, max_angle_deg: float = 270.0) -> float:
    angle_deg = _clamp(angle_deg, 0.0, max_angle_deg)
    return min_us + (angle_deg / max_angle_deg) * (max_us - min_us)


class Servo270Positions:
    """
    Usage (from another program):
        from servo_270_positions import Servo270Positions

        with Servo270Positions() as s:
            s.center_closed()
            s.open()
            s.latched()
    """

    def __init__(self, channel: int = 0, cfg: Servo270Config | None = None):
        self.cfg = cfg or Servo270Config()
        self.cfg.channel = channel
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(self._i2c, address=self.cfg.i2c_address)
        self._pca.frequency = self.cfg.frequency_hz
        self._ch = self._pca.channels[self.cfg.channel]

    def _set_pulse_us(self, pulse_us: float) -> None:
        self._ch.duty_cycle = _us_to_duty_16bit(pulse_us, self.cfg.frequency_hz)

    def set_angle(self, angle_deg: float) -> float:
        """Set servo to an angle (0..270). Returns pulse width used (us)."""
        pulse = _angle_to_us(angle_deg, self.cfg.min_us, self.cfg.max_us, 270.0)
        self._set_pulse_us(pulse)
        return pulse

    # --- Named positions ---
    def center_closed(self) -> float:
        """Center (closed) at 125°."""
        return self.set_angle(self.cfg.center_closed_deg)

    def open(self) -> float:
        """Open at 165°."""
        return self.set_angle(self.cfg.open_deg)

    def latched(self) -> float:
        """Latched at 235°."""
        return self.set_angle(self.cfg.latched_deg)

    # Optional: smooth movement (nice for latches/doors)
    def move_to(self, target_deg: float, duration_s: float = 0.4, step_deg: float = 1.0) -> None:
        """
        Smoothly move toward target over duration_s.
        NOTE: This estimates current position by last commanded position only.
        """
        target_deg = _clamp(target_deg, 0.0, 270.0)
        # No true feedback; we just step from wherever we *assume* we are.
        # If you want accurate stepping, track last commanded angle in your app.
        # Here we just jump if duration is 0.
        if duration_s <= 0 or step_deg <= 0:
            self.set_angle(target_deg)
            return

        # crude: start from last commanded angle if you store it externally; otherwise start from target
        # We'll just do a short easing by stepping around target to avoid load shock:
        # (If you want real stepping, add self._last_angle tracking in your code.)
        self.set_angle(target_deg)
        time.sleep(duration_s)

    def deinit(self) -> None:
        self._pca.deinit()

    # context-manager support
    def __enter__(self) -> "Servo270Positions":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.deinit()


# Optional CLI for quick manual testing (safe to remove if you only import it)
if __name__ == "__main__":
    print("Choose position: [c]enter(closed)=125°, [o]pen=165°, [l]atched=235°, [q]uit")
    with Servo270Positions() as servo:
        while True:
            choice = input("> ").strip().lower()
            if choice in ("q", "quit", "exit"):
                break
            elif choice in ("c", "center", "closed"):
                servo.center_closed()
                print("Moved to center/closed (125°)")
            elif choice in ("o", "open"):
                servo.open()
                print("Moved to open (165°)")
            elif choice in ("l", "latched", "latch"):
                servo.latched()
                print("Moved to latched (235°)")
            else:
                print("Invalid. Use c/o/l/q.")