import cv2
import cv2.aruco as aruco
import numpy as np
import asyncio
import websockets

# Function to calculate distance between two points
def calculate_distance(point1, point2):
    return np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

# Define ArUco marker parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()
#distance threshold
distance_threshold = 50
# IDs of the ArUco markers to track
marker_id1 = 0
marker_id2 = 1

# WebSocket server address
server_address = "ws://localhost:8765"

# Flag to indicate if distance is less than a certain amount
send_flag = False

# Function to send data over WebSocket
async def send_data(websocket, data):
    await websocket.send(data)
    print(f"Sent: {data}")

# Function to handle WebSocket connection
async def websocket_client():
    async with websockets.connect(server_address) as websocket:
        while True:
            # Calculate distance and set send_flag accordingly
            ret, frame = cap.read()
            if not ret:
                break

            corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

            if ids is not None:
                if marker_id1 in ids and marker_id2 in ids:
                    idx_marker1 = np.where(ids == marker_id1)[0][0]
                    idx_marker2 = np.where(ids == marker_id2)[0][0]

                    corners_marker1 = corners[idx_marker1][0]
                    corners_marker2 = corners[idx_marker2][0]

                    center_marker1 = np.mean(corners_marker1, axis=0)
                    center_marker2 = np.mean(corners_marker2, axis=0)

                    distance = calculate_distance(center_marker1, center_marker2)

                    if distance < distance_threshold:  # Adjust the threshold distance as needed
                        send_flag = True
                        await send_data(websocket, "Flag: True")
                    else:
                        send_flag = False

            # Display the frame
            cv2.imshow('Frame', frame)

            # Check for exit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            await asyncio.sleep(0.1)  # Adjust the sleep time as needed

# Create video capture object
cap = cv2.VideoCapture(0)  # Change the parameter to the appropriate camera index if using multiple cameras

# Start the WebSocket client
asyncio.get_event_loop().run_until_complete(websocket_client())
asyncio.get_event_loop().run_forever()

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
