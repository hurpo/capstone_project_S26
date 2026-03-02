import RPi.GPIO as GPIO
import time

IN1, IN2 = 23, 24   # example pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

print("Forward")
GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.LOW)
time.sleep(2)

print("Backward")
GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.HIGH)
time.sleep(2)

print("Stop")
GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.LOW)
