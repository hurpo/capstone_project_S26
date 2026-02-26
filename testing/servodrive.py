#!/usr/bin/env python3
import time
import board
import busio
from adafruit_pca9685 import PCA9685

CHANNEL = 12
FREQ_HZ = 60

# Calibrate these:
STOP_US = 1500          # adjust until it truly stops (try 1470-1530)
MAX_FWD_US = 2000       # one side of STOP
MAX_REV_US = 1000       # the other side of STOP

# Choose direction:
DIRECTION = "forward"   # "forward" or "reverse"

MIN_US = 500
MAX_US = 2500

def set_pulse_us(pca: PCA9685, ch: int, pulse_us: float) -> None:
    pulse_us = max(MIN_US, min(MAX_US, pulse_us))
    period_us = 1_000_000.0 / pca.frequency
    duty = int((pulse_us / period_us) * 65535)
    pca.channels[ch].duty_cycle = duty

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = FREQ_HZ

    # Immediately command stop to prevent “boot drift”
    set_pulse_us(pca, CHANNEL, STOP_US)
    time.sleep(0.2)

    try:
        if DIRECTION == "forward":
            drive_us = MAX_FWD_US
        else:
            drive_us = MAX_REV_US

        print(f"STOP_US={STOP_US}  driving {DIRECTION} at {drive_us}us on CH{CHANNEL}. Ctrl+C to stop.")
        set_pulse_us(pca, CHANNEL, drive_us)

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        # Always stop on exit
        set_pulse_us(pca, CHANNEL, STOP_US)
        time.sleep(0.2)
        pca.deinit()

if __name__ == "__main__":
    main()