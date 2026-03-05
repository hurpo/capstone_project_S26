from calibrate_camera import calibrate_camera
import cv2
from pupil_apriltags import Detector
import numpy as np
import math

ret, mtx, dist, rvecs, tvecs = calibrate_camera()

print(mtx)
print(dist)

test_img = cv2.imread('calibration_images/test00.jpg')
grayimg = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY)

detector = Detector(families='tag36h11')
detections = detector.detect(grayimg)

if len(detections) > 0:
    detect = detections[0]

    april_tag_type = detect.tag_id

    print(f"April Tag ID: {april_tag_type}")

    img_points = detect.corners.astype(np.float32)

    APRIL_TAG_SIZE = 3.15 # Inches
    half = APRIL_TAG_SIZE / 2.0

    obj_points = np.array([
        [0, 20.5, 0.75], # 0
        [0, 23.5, 0.75], # 1
        [0,  23.5, 3.5], # 2
        [0,  20.5, 3.5]  # 3
    ], dtype=np.float32)

    success, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        mtx, 
        dist
    )
    if success:
        print("\nRotation Vector (rvec):\n", rvec)
        print("\nTranslation Vector (tvec):\n", tvec)

        R, _ = cv2.Rodrigues(rvec)

        R_inv = R.T
        camera_position_world = -R_inv @ tvec

        print("\nCamera Position in World Coordinates:")
        print("X:", camera_position_world[0][0])
        print("Y:", camera_position_world[1][0])
        print("Z:", camera_position_world[2][0])

        yaw = math.atan2(R_inv[1,0], R_inv[0,0])
        pitch = math.atan2(-R_inv[2,0], math.sqrt(R_inv[2,1]**2 + R_inv[2,2]**2))
        roll = math.atan2(R_inv[2,1], R_inv[2,2])

        print("\nCamera Yaw Pitch and Roll")
        print("Yaw (deg):", math.degrees(yaw))
        print("Pitch (deg):", math.degrees(pitch))
        print("Roll (deg):", math.degrees(roll))

        # ---- Project 3D points back to image (verification) ----
        projected_points, _ = cv2.projectPoints(
            obj_points,
            rvec,
            tvec,
            mtx,
            dist
        )

        for point in projected_points.astype(int):
            cv2.circle(test_img, tuple(point[0]), 7, (0, 255, 0), -1)

        cv2.imshow('AprilTag Pose Estimation', test_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

else:
    print("No AprilTag detected in test image.")