import sys
from time import sleep
sys.path.insert(1, '../HardwareControls/')
from hardware_classes import Magnetometer, LightSensor

m1 = Magnetometer(0x18)
m2 = Magnetometer(0x19)

led_sensr = LightSensor()

while True:
    print(f"M1: {m1.returnAxisValues()}\nM2: {m2.returnAxisValues()}\nLS: {led_sensr.returnVisible()}\n")
    sleep(0.1)