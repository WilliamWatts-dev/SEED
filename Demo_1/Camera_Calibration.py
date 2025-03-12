import cv2
import numpy as np
import glob
import os

# Define the chessboard dimensions
chessboard_size = (9, 6)
square_size = 22  # millimeters

# Prepare object points based on the real chessboard dimensions
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points
objpoints = []
imgpoints = []

# Get the directory of the script and set the image directory relative to it
script_directory = os.path.dirname(os.path.abspath(__file__))
image_directory = os.path.join(script_directory, "images")
image_format = 'jpg'  # Change this if your images are in a different format

# Create the full file path pattern
image_pattern = os.path.join(image_directory, f'*.{image_format}')

# Load images
images = glob.glob(image_pattern)

if not images:
    print(f"No images found in {image_directory}. Please check the directory path and image format.")
else:
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
            cv2.imshow('Chessboard', img)
            cv2.waitKey(500)
        else:
            print(f"Chessboard corners not found in image: {fname}")

    cv2.destroyAllWindows()

    # Perform camera calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    if ret:
        # Print the camera matrix
        print("Camera matrix:")
        print(camera_matrix)
        print("Dist matrix:")
        print(dist_coeffs)

        # Save the camera matrix and distortion coefficients
        np.savez(os.path.join(script_directory, 'camera_calibration.npz'), 
                 camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
        print("Calibration successful. Calibration data saved to 'camera_calibration.npz'.")
    else:
        print("Calibration failed. Please ensure you have sufficient valid images.")
