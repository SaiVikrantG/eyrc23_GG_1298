import cv2
import cv2.aruco as aruco
import numpy as np

# Function to calculate distance between two points
def calculate_distance(point1, point2):
    return np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

# Define ArUco marker parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters_create()

# IDs of the ArUco markers to track
marker_id1 = 7
marker_id2 = 50

# Load the image from local directory
image_path = "C:/Users/prit4/OneDrive/Desktop/stuff/active_Github_repos/eyrc23_GG_1298/Task_5/aruco_detectiom/sample2.jpg"  # Replace "path/to/your/image.jpg" with the actual path to your image file
image = cv2.imread(image_path)

# Detect ArUco markers in the image
corners, ids, _ = aruco.detectMarkers(image, aruco_dict, parameters=parameters)

if ids is not None:
    # Check if both markers are detected
    if marker_id1 in ids and marker_id2 in ids:
        # Get the indices of the markers in the detected markers list
        idx_marker1 = np.where(ids == marker_id1)[0][0]
        idx_marker2 = np.where(ids == marker_id2)[0][0]

        # Get the corners of the markers
        corners_marker1 = corners[idx_marker1][0]
        corners_marker2 = corners[idx_marker2][0]

        # Calculate the center of each marker
        center_marker1 = np.mean(corners_marker1, axis=0)
        center_marker2 = np.mean(corners_marker2, axis=0)

        # Draw circles at the centers of the markers
        cv2.circle(image, tuple(center_marker1.astype(int)), 5, (0, 255, 0), -1)
        cv2.circle(image, tuple(center_marker2.astype(int)), 5, (0, 255, 0), -1)

        # Calculate and display the distance between the markers
        distance = calculate_distance(center_marker1, center_marker2)
        cv2.putText(image, f"Distance: {distance:.2f} units", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Draw a line connecting the markers
        cv2.line(image, tuple(center_marker1.astype(int)), tuple(center_marker2.astype(int)), (0, 255, 0), 2)

    # Display the image with the line connecting the markers
    cv2.imshow('Image with Line Connecting Markers', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No ArUco markers detected in the image.")
