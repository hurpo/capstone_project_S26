from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

# kit.servo[4].angle = 270
# time.sleep(1)    

kit.continuous_servo[4].throttle = -1
print("Claw On")
time.sleep(1.25)

kit.continuous_servo[4].throttle = 0.1


# kit.servo[4].angle = 180
# time.sleep(1)

# kit.servo[5].angle = 90
# time.sleep(1)

# kit.servo[4].angle = 180
# time.sleep(1)

# kit.servo[4].angle = 0
# time.sleep(0.5)


# kit.servo[5].angle = 0
