import cv2
import cv2.aruco as aruco
import numpy as np
import time

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

a=0
x70, y70 = None, None  # Initialize the position of the moving marker with ID 70
aruco_pos = {}
shortest_path = [20,40,50]
angles = []

dir = {-90:"up",
         0:"right",
        90:"down",
       180:"left"}

def calc_angle(coord1, coord2):
    x2,y2 = coord1
    x1,y1 = coord2
    # if (x2-x1)!=0 :
    return np.degrees(np.arctan((y2-y1)/(x2-x1))), (y2-y1), (x2-x1)

def signal(direction: str):
    if direction=="right":
        print("right turn")
    elif direction=="left":
        print("left turn")
    elif direction=="straight":
        print("straight")

image = cv2.imread('/home/pradhyumna/hardware_round/eyrc23_GG_1298/Task_5A/test3.png')

corners, ids, _ = aruco.detectMarkers(image, aruco_dict)
if ids is not None:
        for i in range(len(ids)):
            print(len(ids))
            marker_id = ids[i][0]
            # if marker_id == 70:
            #     print(i)
            marker_corners = corners[i][0]

            # Assuming each marker is a square, calculate its center
            marker_center = np.mean(marker_corners, axis=0).astype(int)

            # for ptr in 

            # If the detected marker is the moving marker (ID 70), update its initial position
            if marker_id == 70 and x70 is None and y70 is None:
                x70, y70 = marker_center
                # l_top, l_bottom = ()
                # marker_corners = corners[i][0]
                dx = marker_corners[0][0] - marker_corners[3][0]
                dy = marker_corners[0][1] - marker_corners[3][1]
                angle_aruco = np.degrees(np.arctan2(dy, dx))
                angles.append(angle_aruco)
                # print(angle_aruco)
                print(angles)
                orientation = dir[angle_aruco]


            # Calculate distance to the moving marker (ID 70) if x70 and y70 are not None
            if x70 is not None and y70 is not None and marker_id != 70:

                a,dy,dx = calc_angle(marker_center, (x70, y70))
                
                print(f"angle of {marker_id} = {a}")
            
                if a>0 or a<0:

                    if -10>a>-80 or 10<a<80 :
                        agl = 1

                    elif -10<a<10 :
                        agl = 2

                    if -80>a or a>80 :
                        agl = 3
                
                print(agl)

                if agl==1:
                    print(2)
                    print((dy,dx))

                    if orientation=="right":
                        if dy>0:
                            signal("right")
                        elif dy<0:
                            signal("left")

                    elif orientation=="left":
                        if dy>0:
                            signal("left")
                        elif dy<0:
                            signal("right")  

                    elif orientation=="up":
                        if dx>0:
                            signal("right")
                        elif dx<0:
                            signal("left")

                    elif orientation=="down":
                        if dx>0:
                            signal("left")
                        elif dx<0:
                            signal("right")

                if agl==2:
                    
                    signal("straight")

                if agl==3:
                    signal("straight")


cv2.imshow('Detected ArUco Markers', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

