import time
import board
import adafruit_mlx90393
import adafruit_tsl2591
import pupil_apriltags
import argparse
import cv2
import glob
import json
from pathlib import Path
import numpy as np
import math
import threading
import os

class Camera():
    _BASE_DIR = Path(__file__).parent
    APRIL_TAG_CORNERS_3DWRLDPOS = _BASE_DIR / 'april_tag_3dwrld_pos.json'
    CALIBRATION_IMAGES_PATH = _BASE_DIR / 'CalibrationImages/'
    CAMERA_CALIBRATIONS_PATH = _BASE_DIR / 'camera_calibrations.json'

    def __init__(self, robot=None):
        self.robot = robot

        self.cap = None
        self.cap_lock = None

        self.pnp_thread = None
        self.pnp_running = False
        self.pnp_paused = threading.Event()
        self.pnp_paused.set()

        self.capturing_calibration_imgs = False
        self.capture_next_img = False

        self.annotated_frame = None

        self.cap_options = [0, 1, 2]

        self.ret = None
        self.camera_matrix = None
        self.dist_coef = None
        self.rvecs = None
        self.tvecs = None

        try:
            with open(self.CAMERA_CALIBRATIONS_PATH, 'r') as f:
                camera_calibration = json.load(f)

            last_updated = camera_calibration.get('last_updated', 0)
            one_week_ago = time.time() - (7 * 24 * 60 * 60)

            if last_updated > one_week_ago:
                print("Loading cached camera calibration...")
                self.ret = camera_calibration['ret']
                self.camera_matrix = np.array(camera_calibration['camera_matrix'], dtype=np.float32)
                self.dist_coef = np.array(camera_calibration['dist_coef'], dtype=np.float32)
                self.rvecs = [np.array(r, dtype=np.float32) for r in camera_calibration['rvecs']]
                self.tvecs = [np.array(t, dtype=np.float32) for t in camera_calibration['tvecs']]
            else:
                print("Calibration is outdated, recalibrating...")
                self.calibrate()

        except (FileNotFoundError, KeyError):
            print("No calibration file found, recalibrating...")
    
    def start_cam(self):
        print("Starting camera...")
        for path in self.cap_options:
            try:
                self.cap = cv2.VideoCapture(self.cap_options[path])
                if not self.cap.isOpened():
                    raise RuntimeError(f"Failed to Open at {self.cap_options[path]}")
                else:
                    print(f"Successfully Open cam at {path}")
                    return
            except Exception as e:
                print(f"Error: {e}")
                continue
        print("Failed to open camera at any saved path!")
    
    def end_cam(self):
        print("Ending Camera...")

        self.cap.release()
        cv2.destroyAllWindows()
    
    def passed_cam(self, camera, lock=None):
        print("Using server camera...")

        self.cap = camera
        self.cap_lock = lock
    
    def _read_frame(self):
        if self.cap_lock:
            with self.cap_lock:
                return self.cap.read()
        else:
            result, image = self.cap.read()
            image = cv2.flip(image, -1)
            return result, image

    def read_april_tags(self, time_limit=10):

        self.pause_pnp_localization()

        try:
            print("Reading April Tags...")

            detector = pupil_apriltags.Detector(families='tag36h11')
            start_time = time.time()
            while time.time() - start_time < time_limit:
                result, image = self._read_frame()
                
                if not result or image is None:
                    continue
                
                
                
                grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                detections = detector.detect(grayimg)

                for detect in detections:
                    if detect.tag_id is not None:
                        print("tag_id: %s, center: %s" % (detect.tag_id, detect.center))
                        return detect.tag_id
            return "April Tag Failed"
        finally:
            self.resume_pnp_localization()

    def calibrate(self):
        detector = pupil_apriltags.Detector(families='tag36h11')

        objpoints = []
        imgpoints = []

        images = glob.glob(str(self.CALIBRATION_IMAGES_PATH))

        if not images:
            print(f"No calibration images found at {self.CALIBRATION_IMAGES_PATH}")
        
        half = 3.15 / 2.0
        objp = np.array([
            [-half, -half, 0],
            [half, -half, 0],
            [half, half, 0],
            [-half, half, 0]
        ], dtype=np.float32)

        for idx, fname in enumerate(images):
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            detections = detector.detect(gray)

            if detections:
                for detect in detections:
                    imgpoints.append(detect.corners.astype(np.float32))
                    objpoints.append(objp)
        
        if not objpoints:
            print("No tags detected in any images.")
            return
        
        print("Calibrating camera...")

        self.ret, self.camera_matrix, self.dist_coef, self.rvecs, self.tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        camera_calibration_data = {
            "last_updated": time.time(),
            "ret": self.ret,
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coef": self.dist_coef.tolist(),
            "rvecs": [r.tolist() for r in self.rvecs],
            "tvecs": [t.tolist() for t in self.tvecs]
        }

        with open(self.CAMERA_CALIBRATIONS_PATH, 'w') as f:
            json.dump(camera_calibration_data, f, indent=4)

    def start_calibration(self):
        self.pause_pnp_localization()
        if os.path.exists(self.CAMERA_CALIBRATIONS_PATH):
            os.remove(self.CAMERA_CALIBRATIONS_PATH)
        self.ret = None
        self.camera_matrix = None
        self.dist_coef = None
        self.rvecs = None
        self.tvecs = None
        self.capturing_calibration_imgs = True
        self.capture_next_img = False
        self.pnp_thread = threading.Thread(target=self.save_calibration_imgs, daemon=True)
        self.pnp_thread.start()
        print("Calibration capture started!")
    
    def end_calibration(self):
        self.capturing_calibration_imgs = False
        if self.pnp_thread:
            self.pnp_thread.join()  # wait for save_calibration_imgs to finish
        self.resume_pnp_localization()
        print("Calibration capture ended!")
    
    def capture_calibration_img(self):
        self.capture_next_img = True
        print("Capturing calibration image...")

    def save_calibration_imgs(self):
        if not os.path.exists(self.CALIBRATION_IMAGES_PATH):
            os.mkdir(self.CALIBRATION_IMAGES_PATH)

        detector = pupil_apriltags.Detector(families='tag36h11')
        img_count = 0

        try:
            while self.capturing_calibration_imgs:
                ret, frame = self._read_frame()

                if not ret or frame is None:
                    print("Error: Failed to capture image")
                    break
                
                grayimg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                detections = detector.detect(grayimg)
                if detections:
                    for detect in detections:
                        for i, corner in enumerate(detect.corners):
                            print(f"Corner found {corner}")
                
                if self.capture_next_img:
                    self.capture_next_img = False
                    img_name = os.path.join(self.CALIBRATION_IMAGES_PATH, f"test{img_count:02d}.jpg")
                    cv2.imwrite(img_name, frame)
                    print(f"Captured {img_name}")
                    img_count += 1
                    
        except Exception as e:
            print(f"{e}")

    def start_pnp_localization(self):
        self.pnp_thread = threading.Thread(target=self.pnp_localization, daemon=True)
        self.pnp_thread.start()
    
    def stop_pnp_localization(self):
        self.pnp_running = False
    
    def pause_pnp_localization(self):
        print("Pausing pnp localization!")
        self.pnp_paused.clear()
    
    def resume_pnp_localization(self):
        print("pnp localization resumed!")
        self.pnp_paused.set()

    def pnp_localization(self):

        with open(self.APRIL_TAG_CORNERS_3DWRLDPOS, 'r') as f:
            april_tag_data = json.load(f)

        if self.ret is None or self.camera_matrix is None or self.dist_coef is None or self.rvecs is None or self.tvecs is None:
            print("Missing calibration data!?!?!? Recalibrating...")
            self.calibrate()

        detector = pupil_apriltags.Detector(families='tag36h11')
        self.pnp_running = True

        print("pnp localization starting...")
        while self.pnp_running:
            self.pnp_paused.wait()

            result, image = self._read_frame()
            grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(grayimg)

            if detections:
                for detect in detections:
                    april_tag_type = detect.tag_id

                    if april_tag_type in [0, 1, 2, 3, 4]:
                        april_corners = april_tag_data['x']
                    else:
                        april_corners = april_tag_data[str(april_tag_type)]

                    img_points = detect.corners.astype(np.float32)
                    obj_points = np.array([
                        april_corners[0], # 0
                        april_corners[1], # 1
                        april_corners[2], # 2
                        april_corners[3]  # 3
                    ], dtype=np.float32)

                    success, rvec, tvec = cv2.solvePnP(
                        obj_points,
                        img_points,
                        self.camera_matrix, 
                        self.dist_coef
                    )

                    if success:
                        R, _ = cv2.Rodrigues(rvec)
                        R_inv = R.T
                        camera_position_world = -R_inv @ tvec

                        self.robot.updatePosition(dx=camera_position_world[0][0], dy=-camera_position_world[1][0])
                        # print("\nCamera Position in World Coordinates:")
                        # print("X:", camera_position_world[0][0])
                        # print("Y:", camera_position_world[1][0])
                        # print("Z:", camera_position_world[2][0])

                        yaw = math.atan2(R_inv[1,0], R_inv[0,0])
                        pitch = math.atan2(-R_inv[2,0], math.sqrt(R_inv[2,1]**2 + R_inv[2,2]**2))
                        roll = math.atan2(R_inv[2,1], R_inv[2,2])

                        # print("\nCamera Yaw Pitch and Roll")
                        # print("Yaw (deg):", math.degrees(yaw))
                        # print("Pitch (deg):", math.degrees(pitch))
                        # print("Roll (deg):", math.degrees(roll))

                        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

                        projected_points, _ = cv2.projectPoints(
                            obj_points,
                            rvec,
                            tvec,
                            self.camera_matrix, 
                            self.dist_coef
                        )

                        for point in projected_points.astype(int):
                            cv2.circle(image, tuple(point[0]), 7, (0, 255, 0), -1)
                        self.annotated_frame = image

class Magnetometer():
    def __init__(self, address):
        self.i2c = board.I2C()

        try:
            self.SENSOR = adafruit_mlx90393.MLX90393(self.i2c, gain=adafruit_mlx90393.GAIN_1X, address=address)
        except ValueError:
            print(f"Magnetometer ValueError: {ValueError}")
            self.SENSOR = adafruit_mlx90393.MLX90393(self.i2c, gain=adafruit_mlx90393.GAIN_1X, address=address)
    
    def returnAxisValues(self):
        MX, MY, MZ = self.SENSOR.magnetic
        return MX, MY, MZ

class LightSensor():
    def __init__(self, address=0x29):
        self.i2c = board.I2C()

        try:
            self.SENSOR = adafruit_tsl2591.TSL2591(self.i2c, address=address)
        except ValueError:
            print(f"LightSensor ValueError: {ValueError}")
            self.SENSOR = adafruit_tsl2591.TSL2591(self.i2c, address=address)
    
    def returnLux(self):
        return self.SENSOR.lux
    
    def returnInfrared(self):
        return self.SENSOR.infrared
    
    def returnVisible(self):
        return self.SENSOR.visible
    
    def returnFullSpec(self):
        return self.SENSOR.full_spectrum