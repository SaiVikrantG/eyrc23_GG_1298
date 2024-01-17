import cv2
import numpy as np
import time

# Function to calculate Euclidean distance
def calculate_distance(marker1, marker2):
    return np.sqrt((marker2[0] - marker1[0])**2 + (marker2[1] - marker1[1])**2)

# Initialize ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

# Open a connection to the camera (assuming camera index 0, you may need to change it based on your setup)
cap = cv2.VideoCapture(0)

# Example detection loop (adapt based on your actual implementation)
frame_counter = 0  # Counter to track frames

# Variables to store real-time information about marker 70 and the nearest ArUco marker
marker_70_position = None
last_recorded_nearest_marker = None

while True:
    # Capture frame from camera
    ret, frame = cap.read()

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)

    # Check if any markers are detected
    if ids is not None:
        nearest_marker_id = None
        nearest_marker_position = None
        nearest_marker_distance = float('inf')  # Initialize with infinity

        for i in range(len(ids)):
            marker_id = ids[i][0]
            marker_corners = corners[i][0]

            # Assuming each marker is a square, calculate its center
            marker_center = np.mean(marker_corners, axis=0).astype(int)

            # If the detected marker is the moving marker (ID 70), update its position
            if marker_id == 70:
                marker_70_position = marker_center

            # Calculate distance to marker 70 if its position is known
            if marker_70_position is not None and marker_id != 70:
                distance_to_marker_70 = calculate_distance(marker_center, marker_70_position)

                # Update real-time information about the nearest ArUco marker
                if distance_to_marker_70 < nearest_marker_distance:
                    nearest_marker_id = marker_id
                    nearest_marker_position = marker_center
                    nearest_marker_distance = distance_to_marker_70

        # Use the real-time position of marker 70 as its current position
        current_position = marker_70_position

        # Calculate distance to the nearest marker for marker 70
        if marker_70_position is not None and nearest_marker_position is not None:
            distance_to_nearest_marker = calculate_distance(nearest_marker_position, current_position)

            # Check if the nearest marker has changed
            if nearest_marker_id != last_recorded_nearest_marker:
                # Record the new nearest marker
                last_recorded_nearest_marker = nearest_marker_id

                # Do something with the real-time information about marker 70 and the nearest ArUco marker
                print(f"Marker 70 at position {marker_70_position}, "
                      f"Nearest ArUco marker is ID {nearest_marker_id} at position {nearest_marker_position} "
                      f"with distance to marker 70: {distance_to_nearest_marker}")

        # Draw detected markers on the image (optional)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Display the frame with detected markers
    cv2.imshow("Detected ArUco Markers", frame)

    # Increment frame counter
    frame_counter += 1

    # Add a delay of 500 milliseconds for computations
    time.sleep(0.5)

    # Check for key press
    key = cv2.waitKey(1) & 0xFF

    # Break the loop if 'q' is pressed
    if key == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
