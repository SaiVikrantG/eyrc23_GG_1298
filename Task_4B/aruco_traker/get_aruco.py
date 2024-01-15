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
        return [target_marker_corners[0], target_marker_corners[1]]

    return None

# Function to find the nearest ArUco marker and return its ID
# def find_nearest_marker(target_marker_coordinates, all_marker_coordinates, all_marker_ids):
#     distances = np.linalg.norm(all_marker_coordinates - target_marker_coordinates, axis=1)
#     nearest_marker_index = np.argmin(distances)
#     nearest_marker_id = all_marker_ids[nearest_marker_index]
#     return nearest_marker_id

# def find_nearest_marker(target_marker_coordinates, all_marker_coordinates, all_marker_ids):
#     if len(all_marker_ids) > 0:
#         distances = np.linalg.norm(all_marker_coordinates - target_marker_coordinates, axis=1)
#         nearest_marker_index = np.argmin(distances)
#         nearest_marker_id = all_marker_ids[nearest_marker_index]
#         return nearest_marker_id
#     else:
#         return None

def find_nearest_marker(target_marker_coordinates, all_marker_coordinates, all_marker_ids):
    if len(all_marker_ids) > 0:
        distances = np.linalg.norm(all_marker_coordinates - target_marker_coordinates, axis=1)
        nearest_marker_index = np.argmin(distances)
        nearest_marker_id = all_marker_ids[nearest_marker_index] 
        print()
        # if nearest_marker_index < len(all_marker_ids) else None
        return nearest_marker_id
    else:
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
        # Print coordinates of the target marker
        # print(f"Coordinates of ArUco marker {target_marker_id}: {target_marker_coordinates}")

        # Detect all ArUco markers in the frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

        # If other markers are detected, find the nearest one
        if ids is not None and len(ids) > 1:
            # print(f"Detected marker IDs: {ids}")
            # print(f"Detected marker coordinates: {corners}")
            all_marker_coordinates = np.concatenate([corner[0] for corner in corners], axis=0)
            all_marker_ids = ids.flatten()
            # print(f"all_marker_coordinates length: {len(all_marker_coordinates)}")
            # print(f"all_marker_ids length: {len(all_marker_ids)}")

            # Find the nearest ArUco marker ID
            nearest_marker_id = find_nearest_marker(target_marker_coordinates[0], all_marker_coordinates, all_marker_ids)

            # Print the ID of the nearest marker
            if nearest_marker_id != target_marker_id:
                print(f"Nearest ArUco marker ID: {nearest_marker_id}")
        
        

    # Display the frame
    cv2.imshow('Video Frame', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture and close all windows
cap.release()
cv2.destroyAllWindows()
