# Computer Vision script for Demo_2
# Goals:
# Establish communication with Arduino
# Scan for Aruco marker
# Report angle of marker continuously
# Report distance of marker continuously (?)
#
# Extra Considerations:
# Use moving averages
# Calculate yaw to use for alignment purposes
# Calcualte the camera angle for movement purposes
# Yaw should be 0 degrees before the camera angle is 0 degrees
# Use state machine to determine which angle needs alignment
# state 0 - initilzing
# state 1 - marker not detected
# state 2 - marker detected, reporting yaw
# state 3 - marker detected, reproting global angle
# state 4 - marker too  close, reporting turn angle
# Send blocks as (dist, angle[,state,time??])
# TODO I2C thread with simple state machine for decisions
# Use with lock: and global flags to send data over i2c with threading
# 

import cv2
from cv2 import aruco
import numpy as np
from smbus2 import SMBus
import board
from time import sleep
import logging
import threading
from pathlib import Path

# GLOBAL VARS
HEIGHT = 480
WIDTH = 640
ARD_ADDR = 0x08
#LCD_ADDR = 0x20
offset = 0
yaw_angle = 0
camera_angle = 0
distance = 0 

lock = threading.Lock()

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(threadName)s - %(message)s'
)

# Initialize SMBus library with I2C bus 1
bus = SMBUS(1)

# Initialize Aruco detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# Get local path
script_directory = Path(__file__).resolve().parent
calibration_directory = script_directory / "camera_calibration.npz"

# Get camera matrix from camera_calibration.npz
with np.load(calibration_directory) as data:
    camera_matrix = data['camera_matrix.npy']
    dist_coeffs = data['dist_coeffs.npy']
    
# Thread 1: Camera Processing thread
# Capture frame and update yaw_angle, camera_angle, distance, state(,turn_angle & flags)
def camera_thread():
    global yaw_angle, camera_angle, distance, state

    # Initialize camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    print("Press 'q' in any window to quit")

    try: ## TODO move code to seperate threads (thread for i2c etc.)
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
                cv2.imshow('Aruco Detection', dst) # dont need imshow?
                continue

            # Determine index of closest marker
            closest_marker_idx = np.argmin(tvecs[:, 0, 2]) # Replace : with mask filtering out entries that are too close
    
            # Store closest marker into rvec & tvec
            rvec = rvecs[closest_marker_idx]
            tvec = tvecs[closest_marker_idx][0] #[x,y,z]
    













