# Mini Project

# Pi to Arduino pin connections:

import cv2
from cv2 import aruco
import numpy as np
from smbus2 import SMBus
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
import logging

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(threadName)s - %(message)s'
)

# GLOBAL VARS
HEIGHT = 480
WIDTH = 640
ARD_ADDR = 0x08
LCD_ADDR = 0x20
offset = 0


# Message queue for LCD updates
lcd_queue = queue.Queue()

def send_quadrant_to_arduino(bus, quadrant_value):
    max_retries = 3
    retry_delay = 0.1

    for attempt in range(max_retries):
        try:
            bus.write_byte_data(ARD_ADDR, offset, quadrant_value)
            logging.info(f"Successfully sent quadrant value: {quadrant_value}")
            return True
        except Exception as e:
            if attempt < max_retries - 1:
                logging.warning(f"Retry {attempt + 1}/{max_retries} failed: {e}")
                sleep(retry_delay)
            else:
                logging.error(f"Failed to send quadrant after {max_retries} attempts: {e}")
                return False
    return False

def get_quadrants(centers, N):
    quadrants = np.zeros(N, dtype=np.uint8)

    x, y = centers[:, 0], centers[:, 1]  # Extract x and y coordinates

    quadrants[(x >= 0) & (y >= 0)] = 0b10  # SE
    quadrants[(x >= 0) & (y <= 0)] = 0b00  # NE
    quadrants[(x <= 0) & (y >= 0)] = 0b11  # SW
    quadrants[(x <= 0) & (y <= 0)] = 0b01  # NW


    return quadrants

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



# Initialize SMBus library with I2C bus 1
bus = SMBus(1)

# Initialize Aruco detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# Initialize camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise IOError("Cannot open webcam")

# Start LCD thread
lcd_thread = threading.Thread(target=lcd_update_thread, name="LCD-Thread")
lcd_thread.daemon = True  # Thread will close when main program exits
lcd_thread.start()
logging.info("LCD thread started")

# Frame change variables
last_X, last_Y = np.nan, np.nan
last_ids = None

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

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Show IDs when detected
        if ids is not None:
            if not np.array_equal(ids, last_ids):
                id_str = "IDs: " + ",".join(str(id[0]) for id in ids)
                logging.info(id_str)
        else:
            logging.info("No markers found")
        last_ids = ids

        

        
        # Skip marker tracking if not detected
        if ids is None or corners is None:
            cv2.imshow('Aruco Detection', frame)
            continue

        # Draw markers on frame
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Create centers from corners
        centers = np.mean(corners, axis=2)  # centers is (N,1,2) array
        N = centers.shape[0]  # Number of markers detected
        centers = centers.reshape(N, 2)  # (N,1,2) -> (N,2)

        # Draw circle on center
        for x, y in centers.astype(int):
            cv2.circle(frame, (x, y), 15, (255, 0, 0), -1)

        # Shift origin to center of frame
        centers[:, 0] -= WIDTH/2
        centers[:, 1] -= HEIGHT/2
        curr_X = centers[:, 0]
        curr_Y = centers[:, 1]

        # Check if the origin was crossed
        if last_X is np.nan or (last_X * curr_X < 0).any() or (last_Y * curr_Y < 0).any():
            logging.info("Quadrant boundary crossed!")
            quadrants = get_quadrants(centers, N)

            # Send quadrant over I2C
            try:
                if quadrants[0] == 0b00:
                    send_quadrant_to_arduino(bus, 0)
                    logging.info(f"Sent quadrants: {[bin(q) for q in quadrants]}")
                elif quadrants[0] == 0b01:
                    send_quadrant_to_arduino(bus, 1)
                    logging.info(f"Sent quadrants: {[bin(q) for q in quadrants]}")
                elif quadrants[0] == 0b10:
                    send_quadrant_to_arduino(bus, 2)
                    logging.info(f"Sent quadrants: {[bin(q) for q in quadrants]}")
                elif quadrants[0] == 0b11:
                    send_quadrant_to_arduino(bus, 3)
                    logging.info(f"Sent quadrants: {[bin(q) for q in quadrants]}")
            except Exception as e:
                logging.error(f"I2C error: {e}")

            # Queue LCD update
            lcd_queue.put(f"Goal Pos: {bin(quadrants[0])}")

            # Update last positions
            last_X, last_Y = curr_X, curr_Y

        # Display frame
        cv2.imshow('Aruco Detection', frame)


finally:
    # Cleanup
    logging.info("Shutting down...")
    lcd_queue.put("STOP")  # Signal LCD thread to stop
    lcd_thread.join(timeout=1.0)  # Wait for LCD thread to finish
    cap.release()
    cv2.destroyAllWindows()

    

    
# Cleanup
cap.release()
cv2.destroyAllWindows()
#lcd.clear()
#lcd.color = [0, 0, 0]
