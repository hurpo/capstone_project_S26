import pupil_apriltags
import cv2
import argparse
from pupil_apriltags import Detector

Line_length = 5
Center_Color = (0, 255, 0)
Corner_Color = (255, 0, 255)


detector = pupil_apriltags.Detector(families='tag36h11')
cam = cv2.VideoCapture(2)

looping = True

while looping:
    result, image = cam.read()
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
         looping = False

cv2.destroyAllWindows()
cv2.imwrite("final.png", image)
