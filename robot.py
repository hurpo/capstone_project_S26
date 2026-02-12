from HardwareControls.hardware_classes import Magnetometer, LightSensor

class Robot():
    def __init__(self):
        self.LightSensor = LightSensor()
        self.Mag1 = Magnetometer(0x18)
        self.Mag2 = Magnetometer(0x19)
    
    def LEDStart(self, dprint=False):
        print("LED Start ready!")

        while self.LightSensor.returnVisible() <= 6000:
            if dprint:
                print(self.LightSensor.returnVisible())
        if dprint:
            print(self.LightSensor.returnVisible())
        return True