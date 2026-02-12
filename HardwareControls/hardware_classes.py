import time
import board
import adafruit_mlx90393
import adafruit_tsl2591

class Magnetometer():
    def __init__(self, address):
        self.i2c = board.I2C()

        try:
            self.SENSOR = adafruit_mlx90393.MLX90393(self.i2c, gain=adafruit_mlx90393.GAIN_1X, address=address)
        except ValueError:
            print(f"Magnetometer ValueError: {ValueError}")
            self.SENSOR = adafruit_mlx90393.MLX90393(self.i2c, gain=adafruit_mlx90393.GAIN_1X, address=address)
    
    def returnAxisValues(self):
        MX, MY, MZ = self.SENSOR.magnetic
        return MX, MY, MZ

class LightSensor():
    def __init__(self, address=0x29):
        self.i2c = board.I2C()

        try:
            self.SENSOR = adafruit_tsl2591.TSL2591(self.i2c, address=address)
        except ValueError:
            print(f"LightSensor ValueError: {ValueError}")
            self.SENSOR = adafruit_tsl2591.TSL2591(self.i2c, address=address)
    
    def returnLux(self):
        return self.SENSOR.lux
    
    def returnInfrared(self):
        return self.SENSOR.infrared
    
    def returnVisible(self):
        return self.SENSOR.visible
    
    def returnFullSpec(self):
        return self.SENSOR.full_spectrum