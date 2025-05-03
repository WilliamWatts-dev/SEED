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
import struct
import threading
import math
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
bus = SMBus(1)

# Initialize Aruco detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# Initialize camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise IOError("Cannot open webcam")
print("Press 'q' in any window to quit")

# Get local path
script_directory = Path(__file__).resolve().parent
calibration_directory = script_directory / "camera_calibration.npz"

# Get camera matrix from camera_calibration.npz
with np.load(calibration_directory) as data:
    camera_matrix = data['camera_matrix.npy']
    dist_coeffs = data['dist_coeffs.npy']

# Moving average filter for smoothing distance and angle readings
class MovingAverage:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = []
        
    def update(self, value):
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return self.get_average()
    
    def get_average(self):
        if not self.values:
            return 0
        return sum(self.values) / len(self.values)

# Initialize moving averages for distance and angle
distance_avg = MovingAverage()
angle_avg = MovingAverage()

# Function to detect color (red or green)
def detect_color(frame):
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define range for red color (Hue wraps around in HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # Define range for green color
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([90, 255, 255])
    
    # Create masks
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Apply morphological operations to reduce noise
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
    
    # Calculate area of each color
    red_area = cv2.countNonZero(red_mask)
    green_area = cv2.countNonZero(green_mask)
    
    # Determine dominant color (1 for red, 0 for green)
    color_code = 1 if red_area > green_area and red_area > 500 else 0
    
    # For visualization
    red_result = cv2.bitwise_and(frame, frame, mask=red_mask)
    green_result = cv2.bitwise_and(frame, frame, mask=green_mask)
    
    return color_code, red_result, green_result, red_area, green_area

# Function to send data to Arduino
def send_data_to_arduino(distance_feet, angle_degrees, color_code):
    # Pack distance as a 4-byte float
    distance_bytes - struct.pack('f', float(distance_feet))

    # Pack angle as a 4-byte float
    angle_bytes = struct.pack('f', float(angle_degrees))

    # Pack color as a single byte
    color_byte - bytes([color_code & 0x01])

    # Combine data
    combined_data = list(distance_bytes + angle_bytes + color_byte)
    
    try:
        bus.write_i2c_block_data(ARD_ADDR, 0, combined_data)
        logging.info(f"Sent to Arduino: dist={distance_feet}ft, angle={angle_degrees}°, color={'Red' if color_code else 'Green'}")
    except Exception as e:
        logging.error(f"I2C Error: {e}")


# Thread 1: Camera Processing thread
# Capture frame and update yaw_angle, camera_angle, distance, state(,turn_angle & flags)
def camera_thread():
    global yaw_angle, camera_angle, distance, state

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

            # Detect color
            color_code, red_vis, green_vis, red_area, green_area = detect_color(dst)

            # Add color information to display
            cv2.putText(display_frame, f"Red area: {red_area}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display_frame, f"Green area: {green_area}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Color code: {color_code}", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        
            # Catch if marker not detected
            if ids is None or corners is None:
                cv2.imshow('Aruco Detection', dst)
                continue

            # Determine index of closest marker
            closest_marker_idx = np.argmin(tvecs[:, 0, 2])

            # Store closest marker into rvec & tvec
            rvec = rvecs[closest_marker_idx]
            tvec = tvecs[closest_marker_idx][0] #[x,y,z]

            # Draw axes for the closest marker
            cv2.drawFrameAxes(display_frame, camera_matrix, dist_coeffs, 
                         rvec, tvec, 0.03)
            
            # Calculate distance in feet (assuming marker size is in meters)
            # Z-component of tvec is the distance in marker size units
            distance_meters = tvec[2]
            distance_feet = distance_meters * 3.28084  # Convert meters to feet
            
            # Smooth distance with moving average
            smoothed_distance = distance_avg.update(distance_feet)
            
            # Calculate angle (horizontal offset from center)
            # X-component is the horizontal offset
            # Negative angle means target is to the left, positive to the right
            angle_radians = math.atan2(tvec[0], tvec[2])
            angle_degrees = math.degrees(angle_radians)
            
            # Smooth angle with moving average
            smoothed_angle = angle_avg.update(angle_degrees)
            
            # Send data to Arduino
            send_data_to_arduino(smoothed_distance, smoothed_angle, color_code)
            
            # Display info
            cv2.putText(display_frame, f"Marker ID: {marker_id}", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(display_frame, f"Distance: {smoothed_distance:.2f} ft", (10, 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(display_frame, f"Angle: {smoothed_angle:.2f} deg", (10, 180), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        else:
            pass

        cv2.imshow('Aruco Detection', display_frame)
        cv2.imshow('Red Mask', red_vis)
        cv2.imshow('Green Mask', green_vis)

except KeyboardInterrupt:
        print("Program interrupted by user")
except Exception as e:
        logging.error(f"Error: {e}")
finally:
        # Clean up
        cap.release()
        cv2.destroyAllWindows
        print("Program Terminated")




    


