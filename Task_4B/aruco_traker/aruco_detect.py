import cv2
import cv2.aruco as aruco

# Load the image
# image = cv2.imread('/home/pradhyumna/hardware_round/aruco_id.jpeg')

# image = cv2.VideoCapture('http://192.168.0.101:4747/mjpegfeed?640x480')
image = cv2.VideoCapture(2)

# Initialize ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

# Detect ArUco markers
corners, ids, _ = aruco.detectMarkers(image, aruco_dict)

print(ids)
# Draw detected markers on the image (optional)
if ids is not None:
    aruco.drawDetectedMarkers(image, corners, ids)

# Display the result
cv2.imshow('Detected ArUco Markers', image)
# cv2.imwrite(ar_1.jpg)
cv2.waitKey(0)
cv2.destroyAllWindows()
