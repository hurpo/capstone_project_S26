from calibrate_camera import calibrate_camera
import cv2
from pupil_apriltags import Detector
import numpy as np
import math

def get_tag_corners(tag):
    match tag:
        case 5:    # 0,             1,             2,            3
            return [[30, 45, 0.75],  [33, 45, 0.75],  [33, 45, 3.5],  [30, 45, 3.5]]
        case 6:
            return [[44.5, 0, 0.75], [41.5, 0, 0.75], [41.5, 0, 3.5], [44.5, 0, 3.5]]
        case 7:
            return [[93, 23.5, 0.75],[93, 20.5, 0.75],[93, 20.5, 3.5],[93, 23.5, 3.5]]
        case _:
            return [[0, 20.5, 0.75], [0, 23.5, 0.75], [0, 23.5, 3.5], [0, 20.5, 3.5]]

ret, mtx, dist, rvecs, tvecs = calibrate_camera()

detector = Detector(families='tag36h11')
cam = cv2.VideoCapture(0)

while True:
    result, image = cam.read()
    image = cv2.flip(image, -1)
    grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(grayimg)

    if detections:
        for detect in detections:
            april_tag_type = detect.tag_id

            april_corners = get_tag_corners(april_tag_type)

            img_points = detect.corners.astype(np.float32)

            obj_points = np.array([
                [april_corners[0][0], april_corners[0][1], april_corners[0][2]], # 0
                [april_corners[1][0], april_corners[1][1], april_corners[1][2]], # 1
                [april_corners[2][0], april_corners[2][1], april_corners[2][2]], # 2
                [april_corners[3][0], april_corners[3][1], april_corners[3][2]]  # 3
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                mtx, 
                dist
            )

            if success:
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

                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

                projected_points, _ = cv2.projectPoints(
                    obj_points,
                    rvec,
                    tvec,
                    mtx,
                    dist
                )

                for point in projected_points.astype(int):
                    cv2.circle(image, tuple(point[0]), 7, (0, 255, 0), -1)

    cv2.imshow('AprilTag Pose Estimation', image)
    key = cv2.waitKey(100)
    if key == 13:
        break

cv2.destroyAllWindows()

