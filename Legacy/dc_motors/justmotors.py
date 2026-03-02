from __future__ import print_function
from dual_g2_hpmd_rpi import motors, MAX_SPEED
import time

UPLOAD_DATA = 3 
MOTOR_TYPE = 1

class DriverFault(Exception):
    def __init__(self, driver_num):
        self.driver_num = driver_num

test_forward_speeds = list(range(0, MAX_SPEED, 1)) + \
  [MAX_SPEED] * 200 + list(range(MAX_SPEED, 0, -1)) + [0]  

test_reverse_speeds = list(range(0, -MAX_SPEED, -1)) + \
  [-MAX_SPEED] * 200 + list(range(-MAX_SPEED, 0, 1)) + [0] 


try:
    motors.setSpeeds(0, 0)
    time.sleep(3)

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

    print("Turn Left")
    
    # Disable the drivers for half a second.
    motors.disable()
    time.sleep(0.5)
    motors.enable()

except DriverFault as e:
    print("Driver %s fault!" % e.driver_num)

finally:
  # Stop the motors, even if there is an exception
  # or the user presses Ctrl+C to kill the process.
    motors.forceStop()
