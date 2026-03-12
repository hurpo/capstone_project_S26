from smbus2 import SMBus, i2c_msg
import struct
import time
import sys

class M5Module4EncoderMotor:
    ADDR_DEFAULT = 0x24

    REG_PWM_BASE   = 0x20  # int8:  M1..M4
    REG_ENC_BASE   = 0x30  # int32: M1..M4 (4 bytes each)
    REG_SPEED_BASE = 0x40  # int8:  M1..M4
    REG_MOTOR1_BASE = 0x50  # per motor block, +0x10 per motor

    MODE_OPEN_LOOP = 0
    MODE_POSITION  = 1
    MODE_SPEED     = 2

    def __init__(self, bus=1, addr=ADDR_DEFAULT):
        self.addr = addr
        self.bus = SMBus(bus)

    def close(self):
        self.bus.close()

    # ---- raw i2c helpers ----
    def _read_n(self, reg, n):
        w = i2c_msg.write(self.addr, [reg])
        r = i2c_msg.read(self.addr, n)
        self.bus.i2c_rdwr(w, r)
        return bytes(list(r))

    def _write_bytes(self, reg, data: bytes):
        w = i2c_msg.write(self.addr, [reg] + list(data))
        self.bus.i2c_rdwr(w)

    def _write_u8(self, reg, val):
        self._write_bytes(reg, bytes([val & 0xFF]))

    def _write_i8(self, reg, val):
        self._write_bytes(reg, struct.pack(">b", int(val)))  # 1 byte

    def _read_i8(self, reg):
        raw = self._read_n(reg, 1)
        return struct.unpack(">b", raw)[0]

    # ---- public api ----
    def set_pwm(self, motor_index, duty):
        if motor_index not in (1, 2, 3, 4):
            raise ValueError("motor_index must be 1..4")
        duty = max(-127, min(127, int(duty)))
        self._write_i8(self.REG_PWM_BASE + (motor_index - 1), duty)

    def stop_all(self):
        for i in range(1, 5):
            self.set_pwm(i, 0)

    def read_speed(self, motor_index):
        return self._read_i8(self.REG_SPEED_BASE + (motor_index - 1))

    def read_encoder(self, motor_index):
        reg = self.REG_ENC_BASE + 4 * (motor_index - 1)
        raw = self._read_n(reg, 4)
        return struct.unpack(">i", raw)[0]  # BIG-ENDIAN (verified)

    def reset_encoder(self, motor_index=None):
        if motor_index is None:
            for i in range(1, 5):
                self.reset_encoder(i)
            return
        reg = self.REG_ENC_BASE + 4 * (motor_index - 1)
        self._write_bytes(reg, struct.pack(">i", 0))

    def set_mode(self, motor_index, mode):
        base = self.REG_MOTOR1_BASE + 0x10 * (motor_index - 1)
        self._write_u8(base + 0x00, mode)


def clear_line():
    # clears current terminal line
    sys.stdout.write("\r\033[K")

def main():
    # ---------- user settings ----------
    PWM_DUTY = 25              # -127..127 (sign controls direction)
    SAMPLE_DT = 0.20           # seconds between updates
    COUNTS_PER_REV_OUT = None  # set to an int when you measure it (counts per output shaft rev)

    m = M5Module4EncoderMotor(bus=1, addr=0x24)

    try:
        # safe init
        for i in range(1, 5):
            m.set_mode(i, m.MODE_OPEN_LOOP)
        m.reset_encoder()

        # start motors
        for i in range(1, 5):
            m.set_pwm(i, PWM_DUTY)

        # prime previous values
        prev_counts = [m.read_encoder(i) for i in range(1, 5)]
        prev_time = time.time()

        print("Running. Press Ctrl+C to stop.\n")
        print("Format: M#: count (Δcount) [rpm if CPR known] | speed_reg")
        while True:
            time.sleep(SAMPLE_DT)
            now = time.time()
            dt = now - prev_time
            counts = [m.read_encoder(i) for i in range(1, 5)]
            deltas = [counts[i] - prev_counts[i] for i in range(4)]
            speeds = [m.read_speed(i) for i in range(1, 5)]

            parts = []
            for idx in range(4):
                c = counts[idx]
                dc = deltas[idx]
                s = speeds[idx]

                if COUNTS_PER_REV_OUT and COUNTS_PER_REV_OUT > 0:
                    revs = dc / COUNTS_PER_REV_OUT
                    rpm = revs * (60.0 / dt) if dt > 0 else 0.0
                    parts.append(f"M{idx+1}:{c:>11d} ({dc:>8d}) {rpm:>7.1f}rpm | spd:{s:>4d}")
                else:
                    parts.append(f"M{idx+1}:{c:>11d} ({dc:>8d}) | spd:{s:>4d}")

            clear_line()
            sys.stdout.write("   ".join(parts))
            sys.stdout.flush()

            prev_counts = counts
            prev_time = now

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        m.stop_all()
        m.close()
        print("Done.")

if __name__ == "__main__":
    main()
