from gpiozero import AngularServo
from time import sleep

servo = AngularServo(18, min_angle=0, max_angle=180)

servo.angle = 180
sleep(0.5)
servo.angle = 0
sleep(1)
servo.angle = 180