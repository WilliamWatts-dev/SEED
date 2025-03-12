import cv2
import os
from pathlib import Path

# Set up the image directory
script_directory = Path(__file__).resolve().parent
image_directory = script_directory / "images"
image_directory.mkdir(parents=True, exist_ok=True)

# Initialize the webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise IOError("Cannot open webcam")

# Initialize frame counter
frame_count = 0

print("Press 's' to save an image or 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break

    cv2.imshow('Webcam Feed', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        # Save the current frame as an image
        image_path = image_directory / f"image_{frame_count:04d}.jpg"
        cv2.imwrite(str(image_path), frame)
        print(f"Image saved: {image_path}")
        frame_count += 1
    elif key == ord('q'):
        # Exit the loop
        break

# Release the webcam and close windows
cap.release()
cv2.destroyAllWindows()
