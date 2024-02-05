import cv2
import cv2.aruco as aruco
import numpy as np
import websockets
import csv

# Initialize ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

# Open a connection to the camera (assuming camera index 0, you may need to change it based on your setup)
cap = cv2.VideoCapture(2)

# Variables to store real-time information about marker 100 and the nearest ArUco marker
marker_100_position = None
nearest_markers_history = []  # List to store history of nearest markers
req_id = []
count = 0

# Function to calculate Euclidean distance
def calculate_distance(marker1, marker2):
    return np.sqrt((marker2[0] - marker1[0])**2 + (marker2[1] - marker1[1])**2)

def process_csv(file_path):
    data_dict = {}

    with open(file_path, 'r', encoding='utf-8-sig') as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            try:
                ar_id = int(row['id'])
                lat = float(row['lat'])
                lon = float(row['lon'])

                if ar_id not in data_dict:
                    data_dict[ar_id] = [lat, lon]
                else:
                    print(f"Warning: Duplicate ID {ar_id}. Skipping.")

            except (ValueError, KeyError) as e:
                print(f"Error processing row: {row}. {e}")

    return data_dict

csv_file_path = '/home/pradhyumna/hardware_round/eyrc23_GG_1298/Task_5A/task_Integration/lat_long.csv'
processed_data_dict = process_csv(csv_file_path)

def write_csv(loc, csv_name):
    lat, lon = loc 
    with open(csv_name, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        header = ['lat', 'lon'] 
        writer.writerow(header)
        data = [lat, lon]  
        writer.writerow(data)

def qgis_update(ids):
    try:
        lat, lon = processed_data_dict[ids]
        print([ids, lat, lon])
        coordinate = [float(lat), float(lon)]
        write_csv(coordinate, "live_data.csv")
    except KeyError:
        pass 

def websocket_client():
    server_address = "ws://192.168.83.2/ws"
    with websockets.connect(server_address) as websocket:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)

            if ids is not None:
                nearest_marker_id = None
                nearest_marker_position = None
                nearest_marker_distance = float('inf')

                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    marker_corners = corners[i][0]
                    marker_center = np.mean(marker_corners, axis=0).astype(int)

                    if marker_id == 100:
                        marker_100_position = marker_center

                    if marker_100_position is not None and marker_id != 100:
                        distance_to_marker_100 = calculate_distance(marker_center, marker_100_position)

                        if distance_to_marker_100 < nearest_marker_distance:
                            nearest_marker_id = marker_id
                            nearest_marker_position = marker_center
                            nearest_marker_distance = distance_to_marker_100

                current_position = marker_100_position

                if marker_100_position is not None and nearest_marker_position is not None:
                    distance_to_nearest_marker = calculate_distance(nearest_marker_position, current_position)

                    if(distance_to_nearest_marker < 30):
                        if not nearest_markers_history or nearest_marker_id != nearest_markers_history[-1].get("id"):
                            if nearest_marker_id not in [entry.get("id") for entry in nearest_markers_history]:
                                nearest_markers_history.append({
                                    "id": nearest_marker_id,
                                    "position": nearest_marker_position,
                                    "distance_to_marker_100": distance_to_nearest_marker
                                })
                                qgis_update(int(nearest_marker_id))
                                if fil_id_list and fil_id_list[0] == nearest_marker_id:
                                    temp = com_to_send.pop(0)
                                    if temp == 'right':
                                        websocket.send("1")
                                        print("Sent: 1")
                                    elif temp == "left":
                                        websocket.send("2")
                                        print("Sent: 2")
                                    else:
                                        websocket.send("4")
                                        print("Sent: 4")
                                    fil_id_list.pop(0)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow("Overhead Camera Feed", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

# Release the capture
cap.release()
cv2.destroyAllWindows()

if __name__ == "__main__":
    websocket_client()
