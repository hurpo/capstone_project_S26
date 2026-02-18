from smbus2 import SMBus
import time
import struct

I2C_BUS = 1
ADDR = 0x24

# ---- You will fill these in from the module's I2C protocol table ----
REG_MODE = 0x00            # example placeholder
REG_MOTOR_PWM_BASE = 0x20  # example placeholder (per-motor PWM registers)
REG_ENCODER_BASE = 0x30    # example placeholder (per-motor encoder count)
REG_ENCODER_RESET = 0x40   # example placeholder

MODE_DUTY = 0x00
MODE_SPEED = 0x02
MODE_POSITION = 0x01

def write_u8(bus, reg, val):
    bus.write_byte_data(ADDR, reg, val & 0xFF)

def write_block(bus, reg, data_bytes):
    bus.write_i2c_block_data(ADDR, reg, list(data_bytes))

def read_block(bus, reg, n):
    return bytes(bus.read_i2c_block_data(ADDR, reg, n))

def set_mode(bus, mode):
    write_u8(bus, REG_MODE, mode)

def set_motor_duty(bus, motor_index, duty_signed):
    """
    duty_signed: e.g. -1000..+1000 or -100..+100 depending on protocol.
    Sign controls direction (common pattern).
    """
    reg = REG_MOTOR_PWM_BASE + motor_index * 2
    # many devices use little-endian int16
    write_block(bus, reg, struct.pack('<h', int(duty_signed)))

def read_encoder_count(bus, motor_index):
    """
    Commonly a 32-bit signed count, little-endian.
    """
    reg = REG_ENCODER_BASE + motor_index * 4
    raw = read_block(bus, reg, 4)
    return struct.unpack('<i', raw)[0]

def reset_encoders(bus):
    write_u8(bus, REG_ENCODER_RESET, 1)

if __name__ == "__main__":
    with SMBus(I2C_BUS) as bus:
        # 1) set duty mode
        set_mode(bus, MODE_DUTY)

        # 2) reset counts
        reset_encoders(bus)
        time.sleep(0.05)

        # 3) spin motor 1 forward slowly
        set_motor_duty(bus, motor_index=0, duty_signed=200)
        time.sleep(1.0)

        # 4) stop
        set_motor_duty(bus, motor_index=0, duty_signed=0)
        time.sleep(0.2)

        # 5) read encoder
        count = read_encoder_count(bus, motor_index=0)
        print("M1 encoder count:", count)
