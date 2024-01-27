import cv2
import cv2.aruco as aruco
import numpy as np

# Load the image
image = cv2.imread('/home/pradhyumna/hardware_round/eyrc23_GG_1298/Task_5A/test1.png')

# image = cv2.VideoCapture('http://192.168.0.101:4747/mjpegfeed?640x480')
# image = cv2.VideoCapture(2)

# Initialize ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

# Detect ArUco markers
corners, ids, _ = aruco.detectMarkers(image, aruco_dict)
corner_list = []
# print(ids)
# Draw detected markers on the image (optional)
if ids is not None:
    aruco.drawDetectedMarkers(image, corners, ids)
    for i in range(len(ids)):
        corner_list = corners[i][0]
        print(f"corners={corner_list}")
        dx = corner_list[1][0] - corner_list[0][0]
        dy = corner_list[1][1] - corner_list[0][1]
        print(corner_list[1][0])
        angle_degrees = np.degrees(np.arctan2(dy, dx))
    # print(corners[1][0])
        print(angle_degrees)
    

# Display the result
cv2.imshow('Detected ArUco Markers', image)
# cv2.imwrite(ar_1.jpg)
cv2.waitKey(0)
cv2.destroyAllWindows()
