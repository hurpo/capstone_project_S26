#!/usr/bin/env python3
"""
M5Stack Module 4EncoderMotor V1.1 (addr 0x24) - Motor + Encoder Test (Raspberry Pi)

What it does:
- For each motor (M1..M4):
  - Reset encoder to 0 (optional)
  - Run forward at a chosen PWM duty for T seconds
  - Stop, read encoder delta and counts/sec
  - Run reverse, stop, read encoder delta and counts/sec
  - Also reads module-reported speed (int8) if desired

Wiring assumptions:
- Raspberry Pi I2C on bus 1, common GND with module and motor supply
- Motor supply connected to module (6-12V, your case 12V)
- Motors connected to M1..M4 encoder-motor ports on module
"""

import time
import struct
from smbus2 import SMBus

I2C_BUS = 1
ADDR = 0x24

# Register map from your protocol image
REG_PWM_BASE   = 0x20  # int8:  M1..M4 (signed -127..127)
REG_ENC_BASE   = 0x30  # int32: M1..M4 (little-endian, 4 bytes each)
REG_SPEED_BASE = 0x40  # int8:  M1..M4 (signed -127..127), optional read

# Test parameters (tune these)
PWM_TEST_DUTY = 35      # start modest (e.g., 20-50). Range: 0..127
RUN_TIME_S    = 1.5     # seconds per direction
SETTLE_S      = 0.25    # pause after stopping
RESET_EACH    = True    # reset encoder to 0 before each direction test


def clamp_i8(x: int) -> int:
    return max(-127, min(127, int(x)))


def write_i8(bus: SMBus, reg: int, val: int) -> None:
    """Write a signed int8 to a register."""
    b = struct.pack("<b", clamp_i8(val))[0]  # 1 byte
    bus.write_byte_data(ADDR, reg, b)


def read_i8(bus: SMBus, reg: int) -> int:
    """Read a signed int8 from a register."""
    b = bus.read_byte_data(ADDR, reg) & 0xFF
    return struct.unpack("<b", bytes([b]))[0]


def write_i32(bus: SMBus, reg: int, val: int) -> None:
    """Write a signed int32 little-endian to a register (4 bytes)."""
    data = struct.pack("<i", int(val))
    bus.write_i2c_block_data(ADDR, reg, list(data))


def read_i32(bus: SMBus, reg: int) -> int:
    """Read a signed int32 little-endian from a register (4 bytes)."""
    raw = bus.read_i2c_block_data(ADDR, reg, 4)
    return struct.unpack("<i", bytes(raw))[0]


def set_pwm(bus: SMBus, motor: int, duty: int) -> None:
    """
    motor: 1..4
    duty: signed -127..127 (sign = direction)
    """
    if motor not in (1, 2, 3, 4):
        raise ValueError("motor must be 1..4")
    write_i8(bus, REG_PWM_BASE + (motor - 1), duty)


def stop_all(bus: SMBus) -> None:
    for m in (1, 2, 3, 4):
        set_pwm(bus, m, 0)


def enc_reg(motor: int) -> int:
    return REG_ENC_BASE + 4 * (motor - 1)


def speed_reg(motor: int) -> int:
    return REG_SPEED_BASE + (motor - 1)


def reset_encoder(bus: SMBus, motor: int) -> None:
    write_i32(bus, enc_reg(motor), 0)


def read_encoder(bus: SMBus, motor: int) -> int:
    return read_i32(bus, enc_reg(motor))


def read_speed(bus: SMBus, motor: int) -> int:
    return read_i8(bus, speed_reg(motor))


def run_direction_test(bus: SMBus, motor: int, duty: int, run_time_s: float) -> None:
    """
    Runs motor at given duty for run_time_s, then stops.
    Prints encoder delta, counts/sec, and module speed reading.
    """
    if RESET_EACH:
        reset_encoder(bus, motor)
        time.sleep(0.05)

    c0 = read_encoder(bus, motor)
    t0 = time.time()

    set_pwm(bus, motor, duty)
    time.sleep(run_time_s)

    set_pwm(bus, motor, 0)
    time.sleep(SETTLE_S)

    c1 = read_encoder(bus, motor)
    t1 = time.time()

    dc = c1 - c0
    dt = max(1e-6, (t1 - t0))
    cps = dc / dt

    # Optional: module-reported speed int8
    try:
        sp = read_speed(bus, motor)
    except Exception:
        sp = None

    dir_label = "FWD" if duty >= 0 else "REV"
    if sp is None:
        print(f"  {dir_label}: duty={duty:>4}  enc0={c0:>9}  enc1={c1:>9}  dC={dc:>9}  ~{cps:>9.1f} counts/s")
    else:
        print(f"  {dir_label}: duty={duty:>4}  enc0={c0:>9}  enc1={c1:>9}  dC={dc:>9}  ~{cps:>9.1f} counts/s  speedReg={sp:>4}")


def main():
    print("M5Stack 4EncoderMotor Test (addr 0x24)")
    print("Tip: If motors jerk or driver browns out, reduce PWM_TEST_DUTY and/or RUN_TIME_S.\n")

    with SMBus(I2C_BUS) as bus:
        # Safety stop at start
        stop_all(bus)
        time.sleep(0.1)

        # Quick probe: try to read encoder bytes from M1 to confirm comms
        try:
            _ = read_encoder(bus, 1)
        except Exception as e:
            print("ERROR: Could not read encoder from motor 1. Check I2C wiring/address and power.")
            print(e)
            return

        for motor in (1, 2, 3, 4):
            print(f"\n=== Motor M{motor} ===")
            print("Reading initial encoder:", read_encoder(bus, motor))

            # Forward test
            run_direction_test(bus, motor, +PWM_TEST_DUTY, RUN_TIME_S)

            # Reverse test
            run_direction_test(bus, motor, -PWM_TEST_DUTY, RUN_TIME_S)

            # Final stop + final encoder
            set_pwm(bus, motor, 0)
            time.sleep(0.1)
            print("Final encoder:", read_encoder(bus, motor))

        # Safety stop at end
        stop_all(bus)
        print("\nDone. Motors stopped.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Best-effort stop on Ctrl+C
        try:
            with SMBus(I2C_BUS) as bus:
                stop_all(bus)
        except Exception:
            pass
        print("\nInterrupted. Motors stopped.")
