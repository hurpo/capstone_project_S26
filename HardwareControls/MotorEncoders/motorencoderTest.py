from smbus2 import SMBus, i2c_msg
import struct
import time
import sys

ADDR = 0x24
BUS  = 1

REG_PWM_BASE   = 0x20  # M1..M4 int8 R/W
REG_ENC_BASE   = 0x30  # M1..M4 int32 R/W
REG_SPEED_BASE = 0x40  # M1..M4 int8 R
# If you want reset detection, set this to your FW version register from the table:
REG_FW_VERSION = 0xF0  # (example, per your protocol table)

def clear_line():
    sys.stdout.write("\r\033[K")

class M5:
    def __init__(self, bus=BUS, addr=ADDR):
        self.addr = addr
        self.bus = SMBus(bus)

    def close(self):
        self.bus.close()

    def _read_n(self, reg, n):
        w = i2c_msg.write(self.addr, [reg])
        r = i2c_msg.read(self.addr, n)
        self.bus.i2c_rdwr(w, r)
        return bytes(list(r))

    def _write_bytes(self, reg, data: bytes):
        w = i2c_msg.write(self.addr, [reg] + list(data))
        self.bus.i2c_rdwr(w)

    def write_i8(self, reg, val):
        self._write_bytes(reg, struct.pack(">b", int(val)))

    def read_i8(self, reg):
        return struct.unpack(">b", self._read_n(reg, 1))[0]

    def read_u8(self, reg):
        return self._read_n(reg, 1)[0]

    def read_encoder(self, motor_index):
        reg = REG_ENC_BASE + 4*(motor_index-1)
        raw = self._read_n(reg, 4)
        return struct.unpack(">i", raw)[0]  # big-endian verified

    def reset_encoder(self, motor_index):
        reg = REG_ENC_BASE + 4*(motor_index-1)
        self._write_bytes(reg, struct.pack(">i", 0))

    def set_pwm(self, motor_index, duty):
        duty = max(-127, min(127, int(duty)))
        self.write_i8(REG_PWM_BASE + (motor_index-1), duty)

    def get_pwm(self, motor_index):
        # PWM regs are R/W, so readback should match what you set (unless reset happened)
        return self.read_i8(REG_PWM_BASE + (motor_index-1))

    def read_speed(self, motor_index):
        return self.read_i8(REG_SPEED_BASE + (motor_index-1))


def ramp_to(m5: M5, targets, step=2, dt=0.05):
    """
    targets: dict {motor_index: target_duty}
    ramps from current readback PWM to target
    """
    currents = {i: m5.get_pwm(i) for i in targets.keys()}
    done = False
    while not done:
        done = True
        for i, tgt in targets.items():
            cur = currents[i]
            if cur < tgt:
                cur = min(cur + step, tgt)
                done = False
            elif cur > tgt:
                cur = max(cur - step, tgt)
                done = False
            m5.set_pwm(i, cur)
            currents[i] = cur
        time.sleep(dt)


if __name__ == "__main__":
    m5 = M5()

    # Mixed direction example:
    # M1,M2 forward; M3,M4 reverse
    TARGETS = {1: 25, 2: 25, 3: 25, 4: 25}
    SAMPLE_DT = 0.20

    try:
        # reset encoders
        for i in range(1, 5):
            m5.reset_encoder(i)

        # optional: capture firmware version for reset detection
        fw0 = m5.read_u8(REG_FW_VERSION)

        # ramp up instead of step-change
        ramp_to(m5, TARGETS, step=2, dt=0.05)

        prev_counts = [m5.read_encoder(i) for i in range(1,5)]
        prev_t = time.time()

        print("Running mixed directions. Ctrl+C to stop.")
        while True:
            time.sleep(SAMPLE_DT)
            now = time.time()
            dt = now - prev_t

            counts = [m5.read_encoder(i) for i in range(1,5)]
            deltas = [counts[i] - prev_counts[i] for i in range(4)]
            speeds = [m5.read_speed(i) for i in range(1,5)]
            pwms   = [m5.get_pwm(i) for i in range(1,5)]

            # reset detection (optional)
            fw = m5.read_u8(REG_FW_VERSION)
            reset_flag = "" if fw == fw0 else " **FW CHANGED/RESET?**"

            clear_line()
            sys.stdout.write(
                f"M1 {counts[0]:>10d} d{deltas[0]:>7d} pwm{pwms[0]:>4d} spd{speeds[0]:>4d} | "
                f"M2 {counts[1]:>10d} d{deltas[1]:>7d} pwm{pwms[1]:>4d} spd{speeds[1]:>4d} | "
                f"M3 {counts[2]:>10d} d{deltas[2]:>7d} pwm{pwms[2]:>4d} spd{speeds[2]:>4d} | "
                f"M4 {counts[3]:>10d} d{deltas[3]:>7d} pwm{pwms[3]:>4d} spd{speeds[3]:>4d}"
                f"{reset_flag}"
            )
            sys.stdout.flush()

            prev_counts = counts
            prev_t = now

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # ramp down smoothly
        ramp_to(m5, {1:0,2:0,3:0,4:0}, step=3, dt=0.05)
        m5.close()
        print("Done.")
