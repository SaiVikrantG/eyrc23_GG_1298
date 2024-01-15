import cv2
import cv2.aruco as aruco

cap = cv2.VideoCapture(2)
# cap = cv2.VideoCapture('http://192.168.0.101:4747/mjpegfeed?640x480')

# Load the dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

# Create parameters for detector
# parameters = aruco.DetectorParameters_create()

# parameters = aruco.DetectorParameters()

# # Set some parameters (optional)
# parameters.adaptiveThreshWinSizeMax = 23
# parameters.adaptiveThreshWinSizeMin = 3

target_marker_id = 70

while True:
    ret, frame = cap.read()

    # Convert the frame to grayscale
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

    # if ids is not None and target_marker_id in ids:
    #     target_marker_index = ids.tolist().index(target_marker_id)
    #     target_marker_corners = corners[target_marker_index][0]

    #     # Your tracking logic here

    #     # Draw the bounding box around the target marker
    #     frame = aruco.drawDetectedMarkers(frame, corners)

    # Your marker tracking logic here

    cv2.imshow('Video Feed', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
# cv2.waitKey(0)
cv2.destroyAllWindows()
