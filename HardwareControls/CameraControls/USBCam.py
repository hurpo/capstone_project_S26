import pupil_apriltags
import cv2
import argparse
from pupil_apriltags import Detector
import time

camera = None



def start_cam():
    global camera
    print("Starting camera...")

    camera = cv2.VideoCapture(2)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not camera.isOpened():
        print("Error opening camera at /dev/video2")
        return
    print("Camera started!")

def end_cam():
    global camera
    print("Ending Camera...")

    camera.release()
    cv2.destroyAllWindows()

def read_april_tag(time_limit = 5):
    global camera
    print("Reading April Tags...")

    detector = pupil_apriltags.Detector(families='tag36h11')
    searching = True

    start_time = time.time()
    while searching and time.time() - start_time < time_limit:
        # print(f"T: {time.time() - start_time}")
        result, image = camera.read()
        grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(grayimg)

        for detect in detections:
            if detect.tag_id:
                print("tag_id: %s, center: %s" % (detect.tag_id, detect.center))
                #searching = False
                return detect.tag_id
        key = cv2.waitKey(100)
        if key == 13:
            searching = False
    return "April Tag Failed"
