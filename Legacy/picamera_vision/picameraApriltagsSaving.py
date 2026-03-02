import pupil_apriltags
import cv2
import argparse
from pupil_apriltags import Detector
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
cam = cv2.VideoCapture(0)

start_time = time.time()
duration = 90

initial_tag_id = None
initial_tag_center = None
found_initial_tag = False

while time.time() - start_time < duration:
    result, image = cam.read()
    grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(grayimg)
    if not detections:
        print("Nothing")
    else:
        for detect in detections:
            if not found_initial_tag:
                if detect.tag_id < 5:
                    initial_tag_id = detect.tag_id
                    initial_tag_center = detect.center
                    found_initial_tag = True
                    print(f"Initial tag saved! ID: {initial_tag_id}, Center: {initial_tag_center}")
            print("tag_id: %s, center: %s" % (detect.tag_id, detect.center))
            image = plotPoint(image, detect.center, Center_Color)
            image = plotText(image, Center_Color, detect.center, detect.tag_id)
            for corner in detect.corners:
                image = plotPoint(image, corner, Corner_Color)
    cv2.imshow('Result', image)
    key = cv2.waitKey(100)

cv2.destroyAllWindows()
cv2.imwrite("final.png", image)
print("Ending")
if initial_tag_id is not None:
    print(f"The initially saved tag ID was: {initial_tag_id}")
    print(f"The initially saved tag center was: {initial_tag_center}")
else:
    print("No initial tag under 5 was detected.")
