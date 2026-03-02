import pupil_apriltags
import cv2
import argparse
from pupil_apriltags import Detector
from picamera2 import Picamera2
import time 

Line_length = 5
Center_Color = (0, 255, 0)
Corner_Color = (255, 0, 255)

def plotPoint(image, center, color):
    center =  (int(center[0]), int(center[1]))
    image = cv2.line(image, 
                     (center[0] - Line_length, center[1]),
                     (center[0] + Line_length, center[1]),
                     color,
                     3)
    image = cv2.line(image, 
                     (center[0], center[1] - Line_length),
                     (center[0], center[1] + Line_length),
                     color,
                     3)
    return image

def plotText(image, color, center, text):
    center = (int(center[0]) + 4, int(center[1]) - 4)
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

detector = pupil_apriltags.Detector(families='tag36h11')
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (320, 240)}))
picam2.start()

start_time = time.time()
duration = 90

while time.time() - start_time < duration:
    image = picam2.capture_array()
    grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(grayimg)
    if not detections:
        print("Nothing")
    else:
        for detect in detections:
            print("tag_id: %s, center: %s" % (detect.tag_id, detect.center))
            image = plotPoint(image, detect.center, Center_Color)
            image = plotText(image, Center_Color, detect.center, detect.tag_id)
            for corner in detect.corners:
                image = plotPoint(image, corner, Corner_Color)
    cv2.imshow('Result', image)
    key = cv2.waitKey(100)
    if key == 13:
         break
    time.sleep(0.1) 

picam2.stop()
cv2.destroyAllWindows()
cv2.imwrite("final.png", image)
