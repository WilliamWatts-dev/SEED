# Mini Project
# TODO
# [ ] Detect marker
# [ ] Continuously display image, show pos of marker
# [ ] Show wheel status on LCD w threading

import cv2
from cv2 import aruco
import numpy as np
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# LCD initialization
#i2c = board.I2C()
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, 16, 2)
#lcd.clear()
#lcd.color = [100, 0, 0] # red

# Initialize Aruco detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# Initialize camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise IOError("Cannot open webcam")


last_ids = None
print("Press 'q' in any window to quit")
while True:

    # Capture frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    print(corners)
    # Show IDs when detected, or "No markers found" constantly
    if ids is not None:
        if not np.array_equal(ids, last_ids):
            id_str = "IDs: " + ",".join(str(id[0]) for id in ids)
            print(id_str)
            #lcd.clear()
            #lcd.message = id_str
    else:
        print("No markers found")
        #lcd.clear()
        #lcd.message = "No markers found"

    last_ids = ids

    # Draw markers on frame
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

    # Display frame
    cv2.imshow('Aruco Detection', frame)

    # Calculate center

    # Display position of marker
    

    # Break loop with 'q' key
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

    
# Cleanup
cap.release()
cv2.destroyAllWindows()
#lcd.clear()
#lcd.color = [0, 0, 0]
