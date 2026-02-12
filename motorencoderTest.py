from smbus2 import SMBus, i2c_msg
import struct
import time

I2C_BUS = 1          # Raspberry Pi: /dev/i2c-1
ADDR = 0x24          # Module 4EncoderMotor v1.1 default address

REG_PWM_BASE = 0x20  # INT8: M1..M4 duty
REG_ENC_BASE = 0x30  # INT32: M1..M4 encoder counts (4 bytes each)

def clamp_int8(x: int) -> int:
    return max(-127, min(127, int(x)))

class M5_4EncoderMotor:
    def __init__(self, bus_id: int = I2C_BUS, addr: int = ADDR):
        self.addr = addr
        self.bus = SMBus(bus_id)

    def close(self):
        self.bus.close()

    def _read(self, reg: int, length: int) -> bytes:
        # Write register pointer, then read bytes
        write = i2c_msg.write(self.addr, [reg & 0xFF])
        read = i2c_msg.read(self.addr, length)
        self.bus.i2c_rdwr(write, read)
        return bytes(read)

    def _write(self, reg: int, data: bytes | list[int]):
        if isinstance(data, bytes):
            payload = [reg & 0xFF] + list(data)
        else:
            payload = [reg & 0xFF] + [int(b) & 0xFF for b in data]
        self.bus.write_i2c_block_data(self.addr, payload[0], payload[1:])

    def set_pwm(self, motor: int, duty: int):
        """
        Open-loop PWM duty control.
        duty range: -127..127 (sign sets direction)
        motor: 1..4
        """
        if motor not in (1, 2, 3, 4):
            raise ValueError("motor must be 1..4")
        duty = clamp_int8(duty)
        reg = REG_PWM_BASE + (motor - 1)
        # pack signed int8
        self._write(reg, struct.pack("<b", duty))

    def read_encoder(self, motor: int) -> int:
        """
        Read encoder count (signed 32-bit).
        motor: 1..4
        """
        if motor not in (1, 2, 3, 4):
            raise ValueError("motor must be 1..4")
        reg = REG_ENC_BASE + 4 * (motor - 1)
        raw = self._read(reg, 4)
        return struct.unpack("<i", raw)[0]

    def reset_encoder(self, motor: int):
        """
        Encoder registers are R/W in the protocol map; writing 0 resets count.
        """
        if motor not in (1, 2, 3, 4):
            raise ValueError("motor must be 1..4")
        reg = REG_ENC_BASE + 4 * (motor - 1)
        self._write(reg, struct.pack("<i", 0))

if __name__ == "__main__":
    dev = M5_4EncoderMotor()

    try:
        # quick sanity: see if device shows up at 0x24
        # (also run: i2cdetect -y 1)
        dev.reset_encoder(1)

        print("Driving M1 at +60 duty for 3 seconds while printing encoder count...")
        dev.set_pwm(1, 60)

        t0 = time.time()
        while time.time() - t0 < 3.0:
            enc = dev.read_encoder(1)
            print(f"enc={enc}")
            time.sleep(0.1)

        print("Stopping M1")
        dev.set_pwm(1, 0)
        time.sleep(0.2)

        enc_final = dev.read_encoder(1)
        print(f"Final encoder count: {enc_final}")

    finally:
        dev.set_pwm(1, 0)
        dev.close()
