import cv2
import numpy as np
import time
import csv

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
nearest_markers_history = []  # List to store history of nearest markers

# CSV file setup for writing live data
csv_file_path = "live_data.csv"
csv_columns = ["id", "lat", "lon"]

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

            # Check if the nearest marker has changed and is not already in the history
            if not nearest_markers_history or nearest_marker_id != nearest_markers_history[-1].get("id"):
                # Check if the nearest marker is not already in the history list
                if nearest_marker_id not in [entry.get("id") for entry in nearest_markers_history]:
                    # Append the new nearest marker information to the history list
                    nearest_markers_history.append({
                        "id": nearest_marker_id,
                        "position": nearest_marker_position,
                        "distance_to_marker_70": distance_to_nearest_marker
                    })

                    # Update live_data.csv with the latest ArUco ID and its coordinates
                    with open(csv_file_path, mode='w', newline='') as csv_file:
                        writer = csv.DictWriter(csv_file, fieldnames=csv_columns)
                        writer.writeheader()

                        for entry in nearest_markers_history:
                            writer.writerow({
                                "id": entry["id"],
                                "lat": entry["position"][0],  # Assuming X-coordinate is latitude
                                "lon": entry["position"][1]   # Assuming Y-coordinate is longitude
                            })

                    # Do something with the real-time information about marker 70 and the nearest ArUco marker
                    # print(f"Marker 70 at position {marker_70_position}, "
                    #       f"Nearest ArUco marker is ID {nearest_marker_id} at position {nearest_marker_position} "
                    #       f"with distance to marker 70: {distance_to_nearest_marker}")

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

# Print the values stored in the nearest_markers_history list
print("\nValues stored in nearest_markers_history:")
for entry in nearest_markers_history:
    print(entry)

# Release the capture
cap.release()
cv2.destroyAllWindows()
