# Angle_Measurement.py
# Capture frame and undistort image.
# Detect marker and report angle over I2C

import cv2
from cv2 import aruco
import numpy as np
from pathlib import Path

# GLOBAL VARS
HEIGHT = 480
WIDTH = 640
ANGLE_OFFSET = 0.0
MOVING_AVERAGE_SIZE = 5

recent_angles = []

# Get local path
script_directory = Path(__file__).resolve().parent
calibration_directory = script_directory / "camera_calibration.npz"

# Initialize Aruco detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# Get camera matrix from camera_calibration.npz
with np.load(calibration_directory) as data:
    camera_matrix = data['camera_matrix.npy']
    dist_coeffs = data['dist_coeffs.npy']

# Initialize camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise IOError("Cannot open webcam")

try:
    while True:
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

        # Capture frame
        ret, frame = cap.read()
        if not ret:
            break


        dst = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # Convert to grayscale
        gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


        
        # Skip marker tracking if not detected
        if ids is None or corners is None:
            cv2.imshow('Aruco Detection', dst)
            continue

        # Use camera matrix to determine the angle of the marker.
        # arctan(x,z)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        rvec = rvecs[0]
        tvec = tvecs[0][0]

        angle_rad = np.arctan2(tvec[0], tvec[2])
        angle_deg = np.degrees(angle_rad)


        # Apply calibration offset
        calibrated_angle = angle_deg + ANGLE_OFFSET

        # Add to moving average
        recent_angles.append(calibrated_angle)
        if len(recent_angles) > MOVING_AVERAGE_SIZE:
            recent_angles.pop(0)

        # Calculate moving average
        avg_angle = sum(recent_angles) / len(recent_angles)

        cv2.imshow('Aruco Detection', dst)
        
        print("Angle: ", angle_deg)
        
finally:
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
