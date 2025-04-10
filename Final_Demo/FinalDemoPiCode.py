# Final Demo code for computer vision
# Goals: Discuss the best option for i2c (time stamps)
# Only Process frames when we need to
# Split up into threads when applicable
# Send data every 100ms
# angle_queue = queue.Queue(maxsize=1) - important: only use queue for data sharing
# angle_queue.get_nowait() - able to remove stale values before .put(newAngle)
# angle_queue.get() - will wait until angle_queue is available
# Or using global variables and threading.lock()
# Might need to draw out the processing pipeline of the threads
# ChatGPT mentioned using a distrubutor and dispatcher thread to manage sending data by making copies
# You can use queue.get(timeout=0.05)
# Use time.sleep(0.05) to control output
# Too many threads could use up too many resources

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
ARD_ADDR = 0x08
global_state = {
    "angle": 0.0,
    "distance": 0.0,
    "state": 0,
    "color_code": 0
}

# Queues to share data between threads
frame_queue = queue.Queue(maxsize=1) # Most recent frame from camera feed
marker_queue = queue.Queue(maxsize=1) # Copy of frame for marker thread to access
color_queue = queue.Queue(maxsize=1) # copy of frame for color thread to access
marker_results_queue = queue.Queue(maxsize=1) # Dictionary containing marker results
color_results_queue = queue.Queue(maxsize=1) # Dictionary containing color results

# Global Event
stop_event = threading.Event()

# Lock for threading
lock = threading.Lock()

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(threadName)s - %(message)s'
)

# Initialize SMBus library with I2C bus 1
bus = SMBus(1)

# Initialize Aruco detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
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
        [500, 0, 480/2],
        [0, 500, 640/2],
        [0, 0, 1]
    ], dtype=np.float32)
    dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
    logging.info("Using default camera calibration parameters")

def send_data_to_arduino(distance_feet, angle_degrees, color_code):
    # Pack distance as a 4-byte float
    distance_bytes = struct.pack('f', float(distance_feet))

    # Pack angle as a 4-byte float
    angle_bytes = struct.pack('f', float(angle_degrees))

    # Pack color as a single byte
    color_byte = bytes([color_code & 0x01])

    # Combine data
    combined_data = list(distance_bytes + angle_bytes + color_byte)

    logging.info(f"Combined data: {combined_data}")

    try:
        bus.write_i2c_block_data(ARD_ADDR, 0, combined_data)
        logging.info(f"Sent to Arduino: dist={distance_feet}ft, angle={angle_degrees}°, color={'Red' if color_code else 'Green'}")
    except Exception as e:
        logging.error(f"I2C Error: {e}")


def capture_thread():
    """
    Dedicated capture thread. Reads frames from webcam and updates the queue.
    """

    # Initialize camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    try:
        while not stop_event.is_set():
    
            # Break loop with 'q' key
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break

            # Capture frame
            ret, frame = cap.read()
            if not ret:
                logging.error("Failed to grab frame")
                break

            # Update capture_queue
            if capture_queue.full():
                try:
                    capture_queue.get_nowait()  # Drop the old frame
                except queue.Empty:
                    pass
                capture_queue.put(frame) # Update with most recent frame
    finally:
        cap.release()


def distributor_thread():
    """
    Read from the capture queue and feed the same frame into two queues.
    """
    while not stop_event.is_set():

        # get latest frame from queue
        frame = capture_queue.get()
        
        # Create copies to distribute
        frame_copy_marker = frame.copy()
        frame_copy_color = frame.copy()

        # Add copy to marker detection queue
        if marker_queue.full(): # maxsize 1
            try:
                marker_queue.get_nowait()
            except queue.Empty:
                pass
        marker_queue.put(frame_copy_marker)

        # Add copy to color detection queue
        if color_queue.full():
            try:
                color_queue.get_nowait()
            except queue.Empty
                pass
        color_queue.put(frame_copy_color)
        
            
                
def marker_detection_thread():
    """
    Detect marker and compute angle and distance
    idea: update ROI for color_detection_thread to use
    """
    
    while not stop_event.is_set():
        
        # Get frame from distributor
        frame = marker_queue.get()

        # Undistort frame
        dst = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # Convert to grayscale
        gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Check if marker is detected
        if ids is not None and len(corners) > 0:

        # Estimate pose for each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, 0.05, camera_matrix, dist_coeffs)

        # Determine index of closest marker
        closest_marker_idx = np.argmin(tvecs[:, 0, 2])
        marker_id = ids[closest_marker_idx][0]

        # Store closest marker into rvec & tvec
        rvec = rvecs[closest_marker_idx]
        tvec = tvecs[closest_marker_idx][0]  # [x,y,z]

        # Calculate distance in feet (assuming marker size is in meters)
        # Z-component of tvec is the distance in marker size units
        distance_meters = tvec[2]
        distance_feet = distance_meters * 3.28084  # Convert meters to feet

        # Calculate angle (horizontal offset from center)
        # X-component is the horizontal offset
        # Negative angle means target is to the left, positive to the right
        angle_radians = math.atan2(tvec[0], tvec[2])
        angle_degrees = math.degrees(angle_radians)

        # Update angle and distance
        marker_result = {
            "angle": angle_degrees,
            "distance": distance_feet
        }

        # Place result into marker results queue
        if marker_results_queue.full():
            try:
                marker_results_queue.get_nowait()
            except queue.Empty:
                pass
        marker_results_queue.put(marker_result)

        


def color_detection_thread():
    """
    Run color detection on the frame and determine which direction to turn.
    """
    # Get frame from distributor
    frame = color_queue.get()



    try:
        while not stop_event.is_set():
            
    finally:

def dispatcher_thread():
    """
    Gather the latest results from marker and color detection.
    Then update the global shared state using with lock:.
    Maybe also send over i2c from here.
    """


    try:
        while not stop_event.is_set():

    finally:

def communication_thread():
    """
    Get latest data from global and deliver over I2C.
    """


    try:
        while not stop_event.is_set():
            
    finally:
        
def main():

    # Create threads
    threads = [     
        threading.Thread(target=capture_thread, name="CaptureThread", daemon=True),
        threading.Thread(target=distributor_thread, name="DistributorThread", daemon=True),
        threading.Thread(target=marker_detection_thread, name="MarkerDetectionThread", daemon=True),
        threading.Thread(target=color_detection_thread, name="ColorDetectionThread", daemon=True),
        threading.Thread(target=dispatcher_thread, name="DispatcherThread", daemon=True),
        threading.Thread(target=communication_thread, name="CommunicationThread", daemon=True),
    ]
    
    # Start threads
    for t in threads:
        t.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.05)

    except keyboardInterrupt: # Ctrl+C
        stop_event.set()
        
    finally:
        for t in threads:
            t.join()
        
    
if __name__ == "__main__":
    main()

    
