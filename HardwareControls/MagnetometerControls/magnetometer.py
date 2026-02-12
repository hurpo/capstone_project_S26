print("Importing")
import board
import busio
import adafruit_mlx90393
import time

print("i2c")
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)

if i2c.try_lock():
    print("i2c.try_lock() is true")

    try:
        commands = [
            0x80,
            0xF0,
            0x10
        ]

        for cmd in commands:
            try:
                print(f"Sending command: 0x{cmd:02X}")
                i2c.writeto(0x0c, bytes([cmd]))
                time.sleep(0.2)
            except Exception as e:
                print(f"Command failed:",e)
    except Exception as e:
        print("Failed with {e}")
    finally:
        i2c.unlock()

else:
    print("i2c.try_lock() is false")

time.sleep(1)
print("Now Scanning I2C bus")

while not i2c.try_lock():
    print("NOT WORKING!!!")
    pass
addresses = i2c.scan()
i2c.unlock()

print(f"Devices found: {[hex(addr) for addr in addresses]}")

print("Trying....")
try:
    print("Presensor")
    SENSOR = adafruit_mlx90393.MLX90393(i2c, gain=adafruit_mlx90393.GAIN_1X)
    print("After sensor")
except ValueError:
    print("Issue")
    SENSOR = adafruit_mlx90393.MLX90393(i2c, gain=adafruit_mlx90393.GAIN_1X, address=0x0c)

magnetic = False

print("Starting loop")
while True:
    MX, MY, MZ = SENSOR.magnetic
    total_M = (MX * MY * MZ) / 3

    if abs(int(MZ)) >= 1000:
        magnetic = True
    else:
        magnetic = False
    # print(f"MY: {abs(int(MY))}")
    print(f"Magnetic? {magnetic} - {abs(int(MZ))}")

    if SENSOR.last_status > adafruit_mlx90393.STATUS_OK:
        SENSOR.display_status()