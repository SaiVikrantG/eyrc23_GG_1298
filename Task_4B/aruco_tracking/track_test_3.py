import cv2
import numpy as np
import time
import csv
com_to_send = ['right','right']
fil_id_list = [51,39]
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
req_id = []
count = 0

def process_csv(file_path):
    data_dict = {}

    with open(file_path, 'r', encoding='utf-8-sig') as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            try:
                # Access each attribute by its name and convert to the appropriate data type
                ar_id = int(row['id'])
                lat = float(row['lat'])
                lon = float(row['lon'])

                # Create a dictionary for each ID if it doesn't exist
                if ar_id not in data_dict:
                    # data_dict[ar_id] = {'lat': lat, 'lon': lon}
                    data_dict[ar_id] = [lat, lon]
                else:
                    # If the ID already exists, you can decide how to handle duplicates
                    print(f"Warning: Duplicate ID {ar_id}. Skipping.")

            except (ValueError, KeyError) as e:
                print(f"Error processing row: {row}. {e}")

    return data_dict

csv_file_path = 'lat_long.csv'
processed_data_dict = process_csv(csv_file_path)

def write_csv(loc, csv_name):

    # open csv (csv_name)
    # write column names "lat", "lon"
    # write loc ([lat, lon]) in respective columns

    lat,lon = loc 
    with open(csv_name, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        header = ['lat', 'lon'] 
        writer.writerow(header)

        data = [lat,lon]  
        writer.writerow(data)

    
def qgis_update(ids):
    try:
        lat, lon = processed_data_dict[ids]
        print([ids, lat,lon])
        coordinate = [float(lat),float(lon)]
        write_csv(coordinate,"live_data.csv")
    except KeyError:
        pass 

'''cv2.namedWindow('Overhead Camera Feed', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Overhead Camera Feed', 860, 1080)'''

while True:
    # Capture frame from camera
    ret, frame = cap.read()
    # frame1 = frame
    ret1, frame1 = cap.read()
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
                    # req_id.append(int(nearest_marker_id))

                    qgis_update(int(nearest_marker_id))
                    # if fil_id_list.pop(0)==nearest_marker_id:
                    #     temp = com_to_send.pop(0)
                    #     if temp == 'right':
                    #         await websocket.send("1")
                    #         print("Sent: 1")
                    #     else:
                    #         await websocket.send("2")
                    #         print("Sent: 2")

                    # count += 1

                    # Do something with the real-time information about marker 70 and the nearest ArUco marker
                    # print(f"Marker 70 at position {marker_70_position}, "
                    #       f"Nearest ArUco marker is ID {nearest_marker_id} at position {nearest_marker_position} "
                    #       f"with distance to marker 70: {distance_to_nearest_marker}")

    # Draw detected markers on the image (optional)
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Display the frame with detected markers
    # cv2.imshow("Detected ArUco Markers", frame)
    cv2.imshow("Overhead Camera Feed", frame1)


    # Increment frame counter
    frame_counter += 1

    # Add a delay of 500 milliseconds for computations
    # time.sleep(0.5)

    # Check for key press
    key = cv2.waitKey(1) & 0xFF

    # Break the loop if 'q' is pressed
    if key == ord('q'):
        break

# Print the values stored in the nearest_markers_history list
    # print("\nValues stored in nearest_markers_history:")
    # for entry in req_id:
    #     print(entry)

# print(req_id)


# Example usage


# lat, lon = processed_data_dict[23]

# Release the capture
cap.release()
cv2.destroyAllWindows()