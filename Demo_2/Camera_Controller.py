# Computer Vision script for Demo_2
# Goals:
# Establish communication with Arduino
# Scan for Aruco marker
# Report angle of marker continuously
# Report distance of marker continuously (?)
#
# Extra Considerations:
# Use moving averages
# Calculate yaw to determine turn angle

# Send blocks as (dist, angle)

import cv2
from cv2 import aruco
import numpy as np
from smbus2 import SMBus
import board
from time import sleep
import logging


# GLOBAL VARS
HEIGHT = 480
WIDTH = 640
ARD_ADDR = 0x08
#LCD_ADDR = 0x20
offset = 0

# Initialize SMBus library with I2C bus 1
bus = SMBUS(1)

# Initialize Aruco detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# Initialize camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise IOError("Cannot open webcam")
print("Press 'q' in any window to quit")

try:
    while True:
        
        # Break loop with 'q' key
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

        # Capture frame
        ret, frame = cap.read()
        if not ret:
            logging.error("Failed to grab frame")
            break
        
        # Undistort frame
        dst = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # Convert to grayscale
        gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
     
        # Catch if marker not detected
        if ids is None or corners is None:
            cv2.imshow('Aruco Detection', dst)
            continue

        # Determine index of closest marker
        closest_marker_idx = np.argmin(tvecs[:, 0, 2])

        # Store closest marker into rvec & tvec
        rvec = rvecs[closest_marker_idx]
        tvec = tvecs[closest_marker_idx][0] #[x,y,z]



















