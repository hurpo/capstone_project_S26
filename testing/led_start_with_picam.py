import cv2
import numpy as np
from PIL import Image
from picamera2 import Picamera2
import time

lower_range = np.array([120, 100, 100])
upper_range = np.array([160, 255, 255])

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (1920, 1080)}))
picam2.start()

start_time = time.time()
duration = 90

while time.time() - start_time < duration:
    frame = picam2.capture_array()
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

picam2.stop()
cv2.destroyAllWindows()
