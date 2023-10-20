import cv2 
from cv2 import aruco 
import numpy as np 


img = cv2.imread("/home/vikrant/eyrc23-24/Task_2A/Task_2A_files/public_test_cases/aruco_0.png")

cv2.imshow("aruco 1",img)
cv2.waitKey(0)
cv2.destroyAllWindows()

aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters()

corners,ids,rejected = cv2.aruco.detectMarkers(img,aruco_dict,parameters = parameters)

for corner in corners:
    print(corner)