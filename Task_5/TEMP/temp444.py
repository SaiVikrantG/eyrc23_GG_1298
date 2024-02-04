import cv2
import numpy as np

# Define the ArUco dictionary
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

# Create parameters for ArUco detector
arucoParams = cv2.aruco.DetectorParameters_create()

# Open the video source
cap = cv2.VideoCapture(0)

while True:
    # Read a new frame
    ret, frame = cap.read()

    # Convert image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the input frame
    corners, ids, _ = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)

    # Draw detected markers
    frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    # Display the resulting frame
    cv2.imshow('ArUco Markers', frame_markers)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
