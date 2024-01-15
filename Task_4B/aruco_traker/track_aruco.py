import cv2
import cv2.aruco as aruco
import numpy as np

# Function to detect ArUco markers
def detect_aruco_markers(frame, target_marker_id):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    if ids is not None and target_marker_id in ids:
        # Filter corners and ids for the target marker
        marker_index = np.where(ids == target_marker_id)[0][0]
        target_marker_corners = corners[marker_index][0]

        # Draw the bounding box around the target marker
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # Return the coordinates of the top-left corner of the target marker
        return target_marker_corners[0][0]

    return None

# Create a VideoCapture object (0 corresponds to the default camera)
cap = cv2.VideoCapture(2)
# cap = cv2.VideoCapture('http://192.168.0.101:4747/mjpegfeed?640x480')

# ArUco marker ID to track
target_marker_id = 70  # Replace with the desired ArUco marker ID

while True:
    # Read a frame from the video capture
    ret, frame = cap.read()

    # Check if the frame is successfully read
    if not ret:
        break

    # Detect ArUco markers and get the coordinates of the target marker
    target_marker_coordinates = detect_aruco_markers(frame, target_marker_id)

    if target_marker_coordinates is not None:
        print(f"Coordinates of ArUco marker {target_marker_id}: {target_marker_coordinates}")

    # Display the frame
    cv2.imshow('Video Frame', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture and close all windows
cap.release()
cv2.destroyAllWindows()
