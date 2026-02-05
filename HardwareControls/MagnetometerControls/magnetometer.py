print("Importing")
import board
import adafruit_mlx90393

print("i2c")
i2c = board.I2C()

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