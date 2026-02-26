from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)


kit.continuous_servo[1].throttle = 1
print("Throttle on")
time.sleep(5)
kit.continuous_servo[1].throttle = 0.0
print("sleep")
