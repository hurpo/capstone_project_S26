import cv2
import os
import pupil_apriltags
from pupil_apriltags import Detector

CAMERA = 0
OUTPUT_DIR = 'calibration_images'
IMAGE_RES = (1920, 1080)

detector = pupil_apriltags.Detector(families='tag36h11')

def capture_image():
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    cap = cv2.VideoCapture(CAMERA)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_RES[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_RES[1])

    if not cap.isOpened():
        print(f"Error! Couldn't open camera at {CAMERA}")
        return
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera res: {width}x{height}")

    img_count = 0

    print("Press 'c' to capture an image")
    print("Press 'q' or Escape to quit")
    print(f"Images will be saved to {OUTPUT_DIR}")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture image")
            break
        
        grayimg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(grayimg)
        if detections:
            for detect in detections:
                for i, corner in enumerate(detect.corners):
                    print(f"Corner found {corner}")
        
        cv2.imshow('Camera Calibration', frame)

        key = cv2.waitKey(1)

        if key == ord('q') or key == 27:
            break
        
        elif key == ord('c'):
            img_name = os.path.join(OUTPUT_DIR, f"test{img_count:02d}.jpg")
            cv2.imwrite(img_name, frame)
            print(f"Captured {img_name}")

            img_count += 1
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_image()