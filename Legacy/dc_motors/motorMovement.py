from __future__ import print_function
from dual_g2_hpmd_rpi import motors, MAX_SPEED
import time

UPLOAD_DATA = 3 
MOTOR_TYPE = 1

class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num

def move_forward(speed, duration):
    motors.motor1.setSpeed(speed)
    motors.motor2.setSpeed(speed)
    time.sleep(duration)
    motors.setSpeeds(0, 0)

def move_reverse(speed, duration):
    motors.motor1.setSpeed(-speed)
    motors.motor2.setSpeed(-speed)
    time.sleep(duration)
    motors.setSpeeds(0, 0)

def turn_left(speed, duration):
    motors.motor1.setSpeed(-speed)  # Left motor reverse
    motors.motor2.setSpeed(speed)   # Right motor forward
    time.sleep(duration)
    motors.setSpeeds(0, 0)

def turn_right(speed, duration):
    motors.motor1.setSpeed(speed)   # Left motor forward
    motors.motor2.setSpeed(-speed)  # Right motor reverse
    time.sleep(duration)
    motors.setSpeeds(0, 0)

def gradual_acceleration_test():
    test_forward_speeds = list(range(0, MAX_SPEED, 1)) + \
      [MAX_SPEED] * 200 + list(range(MAX_SPEED, 0, -1)) + [0]  

    test_reverse_speeds = list(range(0, -MAX_SPEED, -1)) + \
      [-MAX_SPEED] * 200 + list(range(-MAX_SPEED, 0, 1)) + [0] 

    print("Forward")
    for s in test_forward_speeds:
        motors.motor1.setSpeed(s)
        motors.motor2.setSpeed(s)
        time.sleep(0.002)

    print("Reverse")
    for s in test_reverse_speeds:
        motors.motor1.setSpeed(s)
        motors.motor2.setSpeed(s)
        time.sleep(0.002)

try:
    motors.setSpeeds(0, 0)
    time.sleep(1)

    print("Forward")
    move_forward(MAX_SPEED // 2, 2)
    time.sleep(0.5)

    print("Reverse")
    move_reverse(MAX_SPEED // 2, 2)
    time.sleep(0.5)

    print("Turn Left")
    turn_left(MAX_SPEED // 2, 1)
    time.sleep(0.5)

    print("Turn Right")
    turn_right(MAX_SPEED // 2, 1)
    time.sleep(0.5)

    motors.disable()
    time.sleep(0.5)
    motors.enable()

except DriverFault as e:
    print("Driver %s fault!" % e.driver_num)

finally:
    motors.forceStop()