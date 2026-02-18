#!/usr/bin/env python3
from smbus2 import SMBus, i2c_msg
import struct, time

I2C_BUS=1
ADDR=0x24
REG_PWM_BASE=0x20
REG_ENC_BASE=0x30

def clamp_int8(x): return max(-127, min(127, int(x)))

class Dev:
    def __init__(self): self.bus=SMBus(I2C_BUS)
    def close(self): self.bus.close()
    def _read(self, reg, n):
        w=i2c_msg.write(ADDR,[reg&0xFF])
        r=i2c_msg.read(ADDR,n)
        self.bus.i2c_rdwr(w,r)
        return bytes(r)
    def _write(self, reg, data):
        self.bus.write_i2c_block_data(ADDR, reg&0xFF, list(data))
    def set_pwm(self, m, duty):
        self._write(REG_PWM_BASE+(m-1), struct.pack("<b", clamp_int8(duty)))
    def stop_all(self):
        for m in range(1,5): self.set_pwm(m,0)
    def enc(self, m):
        raw=self._read(REG_ENC_BASE+4*(m-1),4)
        return struct.unpack(">i", raw)[0]   # BIG-endian
    def reset_all(self):
        for m in range(1,5):
            self._write(REG_ENC_BASE+4*(m-1), struct.pack(">i",0))

def main():
    dev=Dev()
    try:
        dev.stop_all()
        dev.reset_all()
        time.sleep(0.2)

        duty=60
        for m_cmd in range(1,5):
            before=[dev.enc(m) for m in range(1,5)]
            dev.stop_all()
            time.sleep(0.1)

            print(f"\nCommanding M{m_cmd} ON for 0.5s")
            dev.set_pwm(m_cmd, duty)
            time.sleep(0.5)
            dev.set_pwm(m_cmd, 0)
            time.sleep(0.2)

            after=[dev.enc(m) for m in range(1,5)]
            delta=[after[i]-before[i] for i in range(4)]
            print("before:", before)
            print(" after:", after)
            print(" delta:", delta)
            moved=[i+1 for i,d in enumerate(delta) if abs(d) > 1]
            print("Encoders that moved:", moved if moved else "None")
    finally:
        try: dev.stop_all()
        except: pass
        dev.close()

if __name__=="__main__":
    main()
