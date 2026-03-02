import cv2
import numpy as np
from PIL import Image

cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

lower_purple = np.array([120, 100, 100])
upper_purple = np.array([160, 255, 255])

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred_frame = cv2.GaussianBlur(hsv_frame, (21, 21), 0)

    purple_mask = cv2.inRange(blurred_frame, lower_purple, upper_purple)

    purple_detection = cv2.bitwise_and(frame, frame, mask=purple_mask)
    
    contours, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
    cv2.imshow("Original Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
