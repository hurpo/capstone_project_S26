import pupil_apriltags
from pupil_apriltags import Detector
import cv2



Line_length = 5
CornerColor = [(255, 0, 255), (255, 255, 255), (0, 255, 255), (0, 0, 255)]

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
    center = (int(center[0]), int(center[1]))
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane.

detector = pupil_apriltags.Detector(families='tag36h11')
cam = cv2.VideoCapture(0)
cv2.calibrateCamera(objpoints, imgpoints, None, None, None)

while True:
    result, image = cam.read()
    grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(grayimg)
    if detections:
        for detect in detections:
            print("#######################################################")
            print(f"0: {detect.corners[0]}\n1: {detect.corners[1]}\n 2: {detect.corners[2]}\n3: {detect.corners[3]}")
            print("#######################################################")
            for i, corner in enumerate(detect.corners):
                image = plotText(image, CornerColor[i], corner, i)
            # image = plotPoint(image, detect.center, Center_Color)
            # image = plotText(image, Center_Color, detect.center, detect.tag_id)
            # for corner in detect.corners:
            #     image = plotPoint(image, corner, Corner_Color)
    cv2.imshow('Result', image)
    key = cv2.waitKey(100)
    if key == 13:
         looping = False

cv2.destroyAllWindows()