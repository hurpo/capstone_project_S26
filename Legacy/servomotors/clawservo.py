from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

kit.servo[0].angle = 90
time.sleep(1)

kit.servo[0].angle = 0
time.sleep(0.5)

kit.servo[0].angle = 180
time.sleep(1)

kit.servo[0].angle = 0
time.sleep(0.5)
#start_time = time.time()
#duration = 90

#while time.time() - start_time < duration:print("Throttle on")

