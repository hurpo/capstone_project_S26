import cv2
import numpy as np
from PIL import Image
from picamera2 import Picamera2
import gpiozero
import time

lower_purple = np.array([120, 100, 100])
upper_purple = np.array([160, 255, 255])

image_width = 320
image_height = 240

picam2 = Picamera2()
config=picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (320, 240)}, controls={"FrameRate": 32} ))

center_image_x = image_width / 2
center_image_y = image_height / 2
minimum_area = 250
maximum_area = 100000

robot = gpiozero.Robot(left=(22, 27), right=(17, 23))
forward_speed = 1.0
turn_speed = 0.8


picam2.start()
start_time = time.time()
duration = 90

while time.time() - start_time < duration:
    frame = picam2.capture_array()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred_frame = cv2.GaussianBlur(hsv_frame, (21, 21), 0)

    purple_mask = cv2.inRange(blurred_frame, lower_purple, upper_purple)

    contours, hierarchy = cv2.findContours(color_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    cv2.imshow("Original Frame", frame)

    object_area = 0
    object_x = 0
    object_y = 0

    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        found_area = width * height
        center_x = x + (width / 2)
        center_y = y + (height / 2)
        if object_area < found_area:
            object_area = found_area
            object_x = center_x
            object_y = center_y
    if object_area > 0:
        object_location = [object_area, object_x, object_y]
    else:
        object_location = None

    # Control the robot based on object location
    if object_location:
        if (object_location[0] > minimum_area) and (object_location[0] < maximum_area):
            if object_location[1] > (center_image_x + (image_width / 3)):
                robot.right(turn_speed)
                print("Turning right")
            elif object_location[1] < (center_image_x - (image_width / 3)):
                robot.left(turn_speed)
                print("Turning left")
            else:
                robot.forward(forward_speed)
                print("Forward")
        elif (object_location[0] < minimum_area):
            robot.stop()
            print("Target isn't large enough, searching")
        else:
            robot.stop()
            print("Target large enough, stopping")
    else:
        robot.stop()
        print("Target not found, searching")
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
