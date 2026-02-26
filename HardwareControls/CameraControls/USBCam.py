import pupil_apriltags
import cv2
import argparse
from pupil_apriltags import Detector
import time

camera = None



def start_cam():
    global camera
    print("Starting camera...")

    camera = cv2.VideoCapture(0)
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

def read_april_tag(cap=None, cap_lock=None, time_limit = 10):
    global camera
    print("Reading April Tags...")

    active_cap = cap if cap is not None else camera

    detector = pupil_apriltags.Detector(families='tag36h11')
    start_time = time.time()
    
    while time.time() - start_time < time_limit:
        if cap_lock:
            with cap_lock:
                result, image = active_cap.read()
        else:
            result, image = active_cap.read()
        
        if not result or image is None:
            continue
        
        grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(grayimg)
        print(f"detections: {detections}")

        for detect in detections:
            print(f"{detect}\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            if detect.tag_id is not None:
                print("tag_id: %s, center: %s" % (detect.tag_id, detect.center))
                return detect.tag_id
        key = cv2.waitKey(100)
        if key == 13:
            break
    return "April Tag Failed"
