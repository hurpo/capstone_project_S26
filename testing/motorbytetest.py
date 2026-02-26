from smbus2 import SMBus, i2c_msg
import time
import struct

ADDR = 0x24
BUS = 1
REG_ENC_M1 = 0x30

def read_bytes(bus, reg, n):
    # Write register pointer, then do a plain I2C read (NOT SMBus block read)
    write = i2c_msg.write(ADDR, [reg])
    read  = i2c_msg.read(ADDR, n)
    bus.i2c_rdwr(write, read)
    return list(read)

def decode(b):
    raw = bytes(b)
    le_i32 = struct.unpack("<i", raw)[0]
    be_i32 = struct.unpack(">i", raw)[0]
    le_u32 = struct.unpack("<I", raw)[0]
    be_u32 = struct.unpack(">I", raw)[0]
    return le_i32, be_i32, le_u32, be_u32

with SMBus(BUS) as bus:
    for k in range(10):
        b = read_bytes(bus, REG_ENC_M1, 4)
        print(k, "raw:", " ".join(f"{x:02X}" for x in b))
        time.sleep(0.1)
        
        le_i32, be_i32, le_u32, be_u32 = decode(b)
        print("LE i32:", le_i32, "BE i32:", be_i32, "LE u32:", le_u32, "BE u32:", be_u32)
