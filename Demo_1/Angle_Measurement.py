# Angle_Measurement.py
# Capture frame and undistort image.
# Determine marker location in space
# Report angle on LCD screen

import cv2
from cv2 import aruco
import numpy as np
from pathlib import Path
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
from time import sleep
import logging

# GLOBAL VARS
HEIGHT = 480
WIDTH = 640
ANGLE_OFFSET = 0.0
MOVING_AVERAGE_SIZE = 5
LCD_ADDR = 0x20

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(threadName)s - %(message)s'
)

# Message queue for LCD updates
lcd_queue = queue.Queue(maxsize=1)

# Store past angles
recent_angles = []

# Get local path
script_directory = Path(__file__).resolve().parent
calibration_directory = script_directory / "camera_calibration.npz"

# Get camera matrix from camera_calibration.npz
with np.load(calibration_directory) as data:
    camera_matrix = data['camera_matrix.npy']
    dist_coeffs = data['dist_coeffs.npy']


def init_i2c_lcd():
    try:
        i2c = board.I2C()
        sleep(0.1) # give time for I2C to initialize

        # create LCD object
        lcd = character_lcd.Character_LCD_RGB_I2C(i2c, 16, 2, address=LCD_ADDR)
        sleep(0.1)

        # Test LCD
        lcd.clear()
        sleep(0.1)
        lcd.color = [100, 0, 0]
        logging.info("LCD initialized successfully")
        return lcd
    except Exception as e:
        logging.error(f"LCD initialization failed: {e}")
        raise


def lcd_update_thread():
    """Thread function to handle LCD updates"""
    lcd = None
    try:
        lcd = init_i2c_lcd()

        while True:
            if not lcd_queue.empty():
                message = lcd_queue.get()
                if message == "STOP":
                    break
                try:
                    lcd.clear()
                    sleep(0.05)
                    lcd.message = message
                    logging.info(f"LCD updated with: {message}")
                except Exception as e:
                    logging.error(f"Error updating LCD: {e}")
            sleep(0.1)  # Small delay to prevent busy-waiting

    except Exception as e:
        logging.error(f"LCD thread error: {e}")
    finally:
        if lcd:
            try:
                lcd.clear()
                lcd.color = [0, 0, 0]
            except Exception as e:
                logging.error(f"LCD cleanup error: {e}")

# Initialize LCD thread
lcd_thread = threading.Thread(target=lcd_update_thread, name="LCD-Thread")
lcd_thread.daemon = True  # Thread will close when main program exits
lcd_thread.start()
logging.info("LCD thread started")

# Initialize Aruco detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

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
            logging.error("Failed to grab frame")
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

        logging.info(f"Angle: {angle_deg}")

        # Queue LCD update
        if not lcd_queue.empty():
            lcd_queue.get_nowait()    
        lcd_queue.put(f"Angle: {angle_deg}")
        
        # Display frame
        cv2.imshow('Aruco Detection', dst)
        
finally:
    # Cleanup
    logging.info("Shutting down...")
    lcd_queue.put("STOP")  # Signal LCD thread to stop
    lcd_thread.join(timeout=1.0)  # Wait for LCD thread to finish
    cv2.destroyAllWindows()
