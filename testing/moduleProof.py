#!/usr/bin/env python3
# Simple: M1+M2 forward, M3+M4 reverse on M5Stack 4EncoderMotor (I2C addr 0x24)

import time
from smbus2 import SMBus

I2C_BUS = 1
ADDR = 0x24
REG_PWM_BASE = 0x20  # M1..M4 int8 duty (-127..127)

def write_pwm(bus, m1, m2, m3, m4):
    # Convert signed int8 to byte (two's complement)
    vals = [m1, m2, m3, m4]
    payload = [(v + 256) % 256 for v in vals]  # int8 -> u8
    bus.write_i2c_block_data(ADDR, REG_PWM_BASE, payload)

def main():
    duty = 40          # change as needed (-127..127)
    run_seconds = 3.0  # how long to run

    with SMBus(I2C_BUS) as bus:
        # M1, M2 forward; M3, M4 reverse
        write_pwm(bus, +duty, +duty, -duty, -duty)
        time.sleep(run_seconds)

        # stop all
        write_pwm(bus, 0, 0, 0, 0)

if __name__ == "__main__":
    main()
