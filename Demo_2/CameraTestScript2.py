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
state = 1  # Initial state: marker not detected

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

# Get local path
script_directory = Path(__file__).resolve().parent
calibration_directory = script_directory / "camera_calibration.npz"

# Get camera matrix from camera_calibration.npz
try:
    with np.load(calibration_directory) as data:
        camera_matrix = data['camera_matrix']
        dist_coeffs = data['dist_coeffs']
    logging.info(f"Loaded camera calibration from {calibration_directory}")
except Exception as e:
    logging.warning(f"Failed to load camera calibration: {e}")
    # Fallback to default values
    camera_matrix = np.array([
        [500, 0, WIDTH/2],
        [0, 500, HEIGHT/2],
        [0, 0, 1]
    ], dtype=np.float32)
    dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
    logging.info("Using default camera calibration parameters")

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
    distance_bytes = struct.pack('f', float(distance_feet))

    # Pack angle as a 4-byte float
    angle_bytes = struct.pack('f', float(angle_degrees))

    # Pack color as a single byte
    color_byte = bytes([color_code & 0x01])

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

            # Create a copy for display
            display_frame = frame.copy()

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

            # Check if marker is detected
            if ids is not None and len(corners) > 0:
                # Draw detected markers
                display_frame = aruco.drawDetectedMarkers(display_frame, corners, ids)

                # Estimate pose for each marker
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 0.05, camera_matrix, dist_coeffs)

                # Determine index of closest marker
                closest_marker_idx = np.argmin(tvecs[:, 0, 2])
                marker_id = ids[closest_marker_idx][0]

                # Store closest marker into rvec & tvec
                rvec = rvecs[closest_marker_idx]
                tvec = tvecs[closest_marker_idx][0]  # [x,y,z]

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

                # Update global variables with lock to prevent race conditions
                with lock:
                    distance = smoothed_distance
                    camera_angle = smoothed_angle
                    yaw_angle = angle_degrees  # You might want to calculate actual yaw from rvec

                    # Update state based on distance
                    if distance < 1.0:  # If closer than 1 foot
                        state = 4  # Marker too close, reporting turn angle
                    else:
                        state = 2  # Marker detected, reporting yaw

                # Send data to Arduino
                send_data_to_arduino(smoothed_distance, smoothed_angle, color_code)

                # Display info
                cv2.putText(display_frame, f"Marker ID: {marker_id}", (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                cv2.putText(display_frame, f"Distance: {smoothed_distance:.2f} ft", (10, 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                cv2.putText(display_frame, f"Angle: {smoothed_angle:.2f} deg", (10, 180), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                cv2.putText(display_frame, f"State: {state}", (10, 210), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            else:
                # No marker detected
                with lock:
                    state = 1  # Marker not detected

            # Show all frames
            cv2.imshow('Aruco Detection', display_frame)
            # cv2.imshow('Grayscale', gray)
            # cv2.imshow('Undistorted', dst)
            cv2.imshow('Red Mask', red_vis)
            cv2.imshow('Green Mask', green_vis)

    except KeyboardInterrupt:
        print("Camera thread interrupted by user")
    except Exception as e:
        logging.error(f"Camera thread error: {e}")
    finally:
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        print("Camera thread terminated")

# Thread 2: Communication thread
# Periodically send data to Arduino
def communication_thread():
    try:
        while True:
            # Get current values with lock to prevent race conditions
            with lock:
                current_distance = distance
                current_angle = camera_angle
                current_state = state
                current_color_code = 0  # You would need to add this as a global variable

            # Only send data if we have valid values
            if current_state != 1:  # If marker is detected
                send_data_to_arduino(current_distance, current_angle, current_color_code)

            # Sleep to avoid flooding the I2C bus
            sleep(0.1)  # 10 Hz update rate

    except KeyboardInterrupt:
        print("Communication thread interrupted by user")
    except Exception as e:
        logging.error(f"Communication thread error: {e}")
    finally:
        print("Communication thread terminated")

# Main function
def main():
    # Create and start camera thread
    camera = threading.Thread(target=camera_thread, name="CameraThread")
    camera.daemon = True

    # Create and start communication thread
    comms = threading.Thread(target=communication_thread, name="CommsThread")
    comms.daemon = True

    # Start threads
    camera.start()
    comms.start()

    try:
        # Keep main thread alive
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("Main thread interrupted by user")
    finally:
        print("Program terminated")

if __name__ == "__main__":
    main()
