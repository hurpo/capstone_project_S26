#!/usr/bin/env python3
"""
Diagnostic for M5Stack 4-Channel Encoder Motor Driver Module (STM32F030) v1.1 on Raspberry Pi.

What it checks:
1) Confirms I2C device is reachable.
2) Reads encoder counts for M1..M4 and prints BOTH:
   - big-endian interpretation (likely correct for this module)
   - little-endian interpretation (for comparison)
3) Runs motors one-at-a-time and then all-at-once while logging encoder deltas.
4) Optional: encoder reset and raw byte dump to spot wiring/noise/endian issues.

Run:
  python3 diag_4encodermotor.py
"""

from smbus2 import SMBus, i2c_msg
import struct
import time

I2C_BUS = 1
ADDR = 0x24

REG_PWM_BASE = 0x20   # int8  : M1..M4 duty
REG_ENC_BASE = 0x30   # int32 : M1..M4 encoder counts (4 bytes each)

def clamp_int8(x: int) -> int:
    return max(-127, min(127, int(x)))

class M5_4EncoderMotor:
    def __init__(self, bus_id=I2C_BUS, addr=ADDR):
        self.addr = addr
        self.bus = SMBus(bus_id)

    def close(self):
        self.bus.close()

    def _read(self, reg: int, n: int) -> bytes:
        w = i2c_msg.write(self.addr, [reg & 0xFF])
        r = i2c_msg.read(self.addr, n)
        self.bus.i2c_rdwr(w, r)
        return bytes(r)

    def _write(self, reg: int, data: bytes):
        self.bus.write_i2c_block_data(self.addr, reg & 0xFF, list(data))

    def ping(self) -> bool:
        # Try to read 1 byte from PWM register; any I2C exception => not reachable
        try:
            _ = self._read(REG_PWM_BASE, 1)
            return True
        except Exception:
            return False

    def set_pwm(self, motor: int, duty: int):
        duty = clamp_int8(duty)
        self._write(REG_PWM_BASE + (motor - 1), struct.pack("<b", duty))

    def stop_all(self):
        for m in range(1, 5):
            self.set_pwm(m, 0)

    def reset_encoder(self, motor: int, endian: str = "be"):
        reg = REG_ENC_BASE + 4 * (motor - 1)
        if endian == "be":
            self._write(reg, struct.pack(">i", 0))
        else:
            self._write(reg, struct.pack("<i", 0))

    def reset_all_encoders(self, endian: str = "be"):
        for m in range(1, 5):
            self.reset_encoder(m, endian=endian)

    def read_encoder_raw(self, motor: int) -> bytes:
        reg = REG_ENC_BASE + 4 * (motor - 1)
        return self._read(reg, 4)

    def read_encoder_be(self, motor: int) -> int:
        return struct.unpack(">i", self.read_encoder_raw(motor))[0]

    def read_encoder_le(self, motor: int) -> int:
        return struct.unpack("<i", self.read_encoder_raw(motor))[0]

def fmt_raw(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def read_all(dev: M5_4EncoderMotor):
    """
    Returns tuples for each motor:
      (raw_bytes, be_value, le_value)
    """
    out = []
    for m in range(1, 5):
        raw = dev.read_encoder_raw(m)
        be = struct.unpack(">i", raw)[0]
        le = struct.unpack("<i", raw)[0]
        out.append((raw, be, le))
    return out

def log_encoders(dev: M5_4EncoderMotor, seconds=2.0, poll=0.05):
    """
    Logs raw bytes + BE/LE values + BE deltas each poll.
    Using BE deltas for “motion” since that’s the likely correct interpretation.
    """
    t0 = time.time()
    prev_be = [dev.read_encoder_be(m) for m in range(1, 5)]

    while time.time() - t0 < seconds:
        samples = read_all(dev)

        be_vals = [s[1] for s in samples]
        le_vals = [s[2] for s in samples]
        raws    = [s[0] for s in samples]

        d_be = [be_vals[i] - prev_be[i] for i in range(4)]
        prev_be = be_vals

        print("RAW:", [fmt_raw(r) for r in raws])
        print(" BE:", be_vals, " dBE:", d_be)
        print(" LE:", le_vals)
        print("-" * 80)

        time.sleep(poll)

def main():
    dev = M5_4EncoderMotor()
    try:
        print(f"I2C ping @ 0x{ADDR:02X} on bus {I2C_BUS}:", "OK" if dev.ping() else "FAILED")
        if not dev.ping():
            print("Device not reachable. Check SDA/SCL, GND, power, and that I2C is enabled.")
            return

        dev.stop_all()
        time.sleep(0.2)

        # Reset encoders assuming big-endian register write (most consistent with your earlier data)
        dev.reset_all_encoders(endian="be")
        time.sleep(0.2)

        print("\n=== Baseline (motors OFF) ===")
        log_encoders(dev, seconds=0.5, poll=0.1)

        duty = 40

        # print("\n=== One motor at a time ===")
        # for m in range(1, 5):
        #     print(f"\nM{m} ON (duty={duty})")
        #     dev.stop_all()
        #     dev.reset_encoder(m, endian="be")
        #     time.sleep(0.15)
        #     dev.set_pwm(m, duty)
        #     log_encoders(dev, seconds=2.0, poll=0.05)
        #     dev.set_pwm(m, 0)
        #     time.sleep(0.3)

        print("\n=== All motors ON ===")
        dev.reset_all_encoders(endian="be")
        time.sleep(0.15)
        # for m in range(1, 5):
        #     dev.set_pwm(m, duty)
        dev.set_pwm(1, duty)
        dev.set_pwm(2, duty)
        dev.set_pwm(3, -duty)
        dev.set_pwm(4, -duty)
        log_encoders(dev, seconds=5.0, poll=0.05)
        dev.stop_all()

        print("\nDone.")
    finally:
        try:
            dev.stop_all()
        except Exception:
            pass
        dev.close()

if __name__ == "__main__":
    main()
