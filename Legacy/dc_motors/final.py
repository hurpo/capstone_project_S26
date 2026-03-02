from __future__ import print_function
from dual_g2_hpmd_rpi import motors, MAX_SPEED
import time


from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

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
        
def roomba():
    kit.continuous_servo[0].throttle = 0.5
    print("Servo 1 Throttle on")

    kit.continuous_servo[1].throttle = -0.5
    print("Servo 2 Throttle on")
    
    print("Forward")
    move_forward(48, 1.25)
    time.sleep(0.5)
    
    kit.continuous_servo[0].throttle = 0.1
    kit.continuous_servo[1].throttle = 0.1
    
    print("Turn Left")
    turn_left(60, 1.6)
    time.sleep(0.5)
    
    kit.continuous_servo[0].throttle = 0.5
    kit.continuous_servo[1].throttle = -0.5
    
    print("Forward")
    move_forward(48, 3)
    time.sleep(0.5)
    
    kit.continuous_servo[0].throttle = 0.1
    kit.continuous_servo[1].throttle = 0.1
    
    print("Turn Right")
    turn_right(60, 1.4)
    time.sleep(0.5)
    
    kit.continuous_servo[0].throttle = 0.5
    kit.continuous_servo[1].throttle = -0.5
    
    print("Forward")
    move_forward(48, 1.8)
    time.sleep(0.5)
    
    kit.continuous_servo[0].throttle = 0.1
    kit.continuous_servo[1].throttle = 0.1
    
    print("Turn Right")
    turn_right(60, 1.21)
    time.sleep(0.5)
    
    kit.continuous_servo[0].throttle = 0.5
    kit.continuous_servo[1].throttle = -0.5
    
    print("Forward")
    move_forward(48, 8)
    time.sleep(0.5)
    
    kit.continuous_servo[0].throttle = 0.1
    kit.continuous_servo[1].throttle = 0.1
    
    
def beacon():
    kit.continuous_servo[4].throttle = -1
    
    kit.servo[5].angle=10
    
    kit.continuous_servo[4].throttle = 0.1
    
    kit.continuous_servo[4].throttle = 1
    print("Claw On")
    time.sleep(0.5)

    kit.continuous_servo[4].throttle = 0.1
    
    print("Reverse")
    move_reverse(60 , 2.5)
    time.sleep(0.5)
    
    kit.servo[5].angle = 90
    time.sleep(1) 
    

    print("Forward")
    move_forward(MAX_SPEED // 8, 2)
    time.sleep(0.5)
    
    
    kit.servo[5].angle=10
    kit.continuous_servo[4].throttle = -1
    
    kit.continuous_servo[4].throttle = 0.1

def bin():
    kit.continuous_servo[4].throttle = -1
    
    kit.servo[5].angle=10
    
    kit.continuous_servo[4].throttle = 0.1
    
    kit.continuous_servo[4].throttle = 1
    print("Claw On")
    time.sleep(0.5)

    kit.continuous_servo[4].throttle = 0.1
    
    print("Reverse")
    move_reverse(60 , 2.5)
    time.sleep(0.5)
    
    kit.servo[5].angle = 90
    time.sleep(1)
    
    print("Forward")
    move_forward(60 , 0.5)
    time.sleep(0.5)
    
    print("Left")
    turn_left(60 , 0.75)
    time.sleep(2)
    
    
    kit.servo[5].angle=10
    kit.continuous_servo[4].throttle = -1
    
    kit.continuous_servo[4].throttle = 0.1

try:
    motors.setSpeeds(0, 0)
    time.sleep(1)
    
    #beacon()
    
    #ROOMBA
    #roomba()
    
    bin()

    
    motors.disable()
    time.sleep(1)
    motors.enable()

except DriverFault as e:
    print("Driver %s fault!" % e.driver_num)

finally:
    motors.forceStop()
