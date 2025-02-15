# Mini Project
# TODO
# [ ] Detect marker
# [ ] Continuously display image, show pos of marker
# [ ] Show wheel status on LCD w threading

import cv2
from cv2 import aruco
import numpy as np
from smbus2 import SMBus
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

def get_quadrants(centers, N):
    quadrants = [''] * N
    
    for center in range(N):
        x, y = centers[center]
        
        if x >= 0 and y >= 0:
            quadrants[center] = 0b10  # SE
        elif x >= 0 and y <= 0:
            quadrants[center] = 0b00  # NE
        elif x <= 0 and y >= 0:
            quadrants[center] = 0b11  # SW
        elif x <= 0 and y <= 0:
            quadrants[center] = 0b01  # NW
            
    return quadrants

# GLOBAL VARS
HEIGHT = 480
WIDTH = 640
ARD_ADDR = 8
offset = 0

# LCD initialization
#i2c = board.I2C()
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, 16, 2)
#lcd.clear()
#lcd.color = [100, 0, 0] # red

# Initialize SMBus library with I2C bus 1
bus = SMBus(1)

# Initialize Aruco detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# Initialize camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise IOError("Cannot open webcam")


last_ids = None
print("Press 'q' in any window to quit")
# Frame change variables
last_X, last_Y = np.nan, np.nan
while True:

    # Break loop with 'q' key
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

    # Capture frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # Show IDs when detected, or "No markers found" constantly
    id_str = ''
    if ids is not None:
        if not np.array_equal(ids, last_ids):
            id_str = "IDs: " + ",".join(str(id[0]) for id in ids)
            print(id_str)

    else:
        print("No markers found")

    last_ids = ids

    # Skip marker tracking if not detected
    if not ids or not corners:
        cv2.imshow('Aruco Detection', frame)
        continue


    # Draw markers on frame
    aruco.drawDetectedMarkers(frame, corners, ids)

    # Create centers from corners
    centers = np.mean(corners, axis=2) # centers is (N,1,2) array
    N = centers.shape[0] # Number of markers detected
    centers = centers.reshape(N,2) # (N,1,2) -> (N,2)

    # Draw circle on center (rec: disable)
    for x,y in centers.astype(int):
        cv2.circle(frame, (x,y), 15, (255,0,0), -1)

    # Shift origin to center of frame
    centers[:, 0] -= WIDTH/2
    centers[:, 1] -= HEIGHT/2

    curr_X = centers[:, 0]
    curr_Y = centers[:, 1]

    # Check if the origin was crossed
    if last_X is np.nan or (last_X * curr_X < 0).any() or (last_Y * curr_Y < 0).any():
        print("Quadrant boundary crossed!")

        quadrants = get_quadrants(centers, N)

        ##print(bin(quadrants[0]))

        # Send quadrant over I2C
        #bus.write_byte_data(ARD_ADDR, offset, quadrants[0])
        #bus.write_block_data(ARD_ADDR, offset, quadrants) # Send block for multiple markers

        # Update LCD with threading
        #lcd.clear()
        #lcd.message = "Goal Position: " + bin(quadrants[0])
    
        # Update last_X, last_Y
        last_X, last_Y = curr_X, curr_Y

                
    # Display frame
    cv2.imshow('Aruco Detection', frame)

    

    
# Cleanup
cap.release()
cv2.destroyAllWindows()
#lcd.clear()
#lcd.color = [0, 0, 0]
