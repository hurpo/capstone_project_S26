import cv2
import numpy as np
from PIL import Image
from picamera2 import Picamera2
import time

lower_range = np.array([120, 100, 100])
upper_range = np.array([160, 255, 255])

cap = cv2.VideoCapture(2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

cap.set(cv2.CAP_PROP_SATURATION, 0)

if not cap.isOpened():
    print("Error opening camera at /dev/video2")


start_time = time.time()
duration = 90

print("All cv2.CAP_PROP properties:")
print("-" * 50)

cap_props = [attr for attr in dir(cv2) if attr.startswith('CAP_PROP_')]

for prop in sorted(cap_props):
    prop_id = getattr(cv2, prop)
    print(f"{prop:40s} = {prop_id}")

while time.time() - start_time < duration:

    ret, frame = cap.read()

    if not ret:
        print("Error, failed to capture frame")
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred_frame = cv2.GaussianBlur(hsv_frame, (21, 21), 0)

    color_mask = cv2.inRange(blurred_frame, lower_range, upper_range)
    color_detection = cv2.bitwise_and(frame, frame, mask=color_mask)

    mask_ = Image.fromarray(color_detection)

    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox
        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

    cv2.imshow("Original Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
