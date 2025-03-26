# Computer Vision script for Demo_2
# Goals:
# Establish communication with Arduino
# Scan for Aruco marker
# Report angle of marker continuously
# Report distance of marker continuously (?)
#
# Extra Considerations:
# Use moving averages
# Marker

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
LCD_ADDR = 0x20
offset = 0

# Initialize SMBus library with I2C bus 1
bus = SMBUS(1)
