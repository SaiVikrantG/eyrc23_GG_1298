import cv2
import numpy as np

# Function to calculate Euclidean distance
def calculate_distance(marker1, marker2):
    return np.sqrt((marker2[0] - marker1[0])**2 + (marker2[1] - marker1[1])**2)

# Initialize ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

# Open a connection to the camera (assuming camera index 0, you may need to change it based on your setup)
cap = cv2.VideoCapture(0)

# Example detection loop (adapt based on your actual implementation)
frame_counter = 0  # Counter to track frames

# Variables to store real-time information about the nearest ArUco marker
nearest_marker_id = None
nearest_marker_position = None
nearest_marker_distance = float('inf')  # Initialize with infinity
x70, y70 = None, None  # Initialize the position of the moving marker with ID 70

while True:
    # Capture frame from camera
    ret, frame = cap.read()

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)

    # Check if any markers are detected
    if ids is not None:
        for i in range(len(ids)):
            marker_id = ids[i][0]
            marker_corners = corners[i][0]

            # Assuming each marker is a square, calculate its center
            marker_center = np.mean(marker_corners, axis=0).astype(int)

            # If the detected marker is the moving marker (ID 70), update its initial position
            if marker_id == 70 and x70 is None and y70 is None:
                x70, y70 = marker_center

            # Calculate distance to the moving marker (ID 70) if x70 and y70 are not None
            if x70 is not None and y70 is not None and marker_id != 70:
                distance_to_marker_70 = calculate_distance(marker_center, (x70, y70))

                # Update real-time information about the nearest ArUco marker
                if distance_to_marker_70 < nearest_marker_distance:
                    nearest_marker_id = marker_id
                    nearest_marker_position = marker_center
                    nearest_marker_distance = distance_to_marker_70

        # Use the last known position of marker 70 as its current position
        current_position = (x70, y70)

        # Calculate distance to the moving marker (ID 70) for the nearest marker
        if x70 is not None and y70 is not None:
            nearest_marker_to_70_distance = calculate_distance(nearest_marker_position, current_position)

            # Do something with the real-time information about the nearest ArUco marker
            print(f"Nearest ArUco marker is ID {nearest_marker_id} at position {nearest_marker_position} "
                  f"with distance to moving marker (ID 70): {nearest_marker_to_70_distance}")

        # Draw detected markers on the image (optional)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Display the frame with detected markers
    cv2.imshow("Detected ArUco Markers", frame)

    # Increment frame counter
    frame_counter += 1

    # Check for key press
    key = cv2.waitKey(1) & 0xFF

    # Break the loop if 'q' is pressed
    if key == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
