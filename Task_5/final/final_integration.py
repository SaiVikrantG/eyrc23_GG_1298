server_address = "ws://192.168.0.112/ws"
import cv2
import cv2.aruco as aruco
import numpy as np
import asyncio
import websockets
import numpy as np
import tensorflow_hub as hub
import tensorflow as tf
import math
import heapq
import matplotlib.pyplot as plt
import csv
###########################csv file path####################
csv_file_path = '/home/pradhyumna/hardware_round/eyrc23_GG_1298/Task_5A/task_Integration/lat_long.csv'

#############################################variables##################################
event_list = ['A','B','D','C','_','E']
# event_list = []
prioriy_order = ['fire','destroyed_buildings','human_aid_rehabilitation','military_vehicles','combat']
detected_list = {}
detected_list_final = {}
frame = "" 
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

nearby_aruco = [7,21,24,25,22,49,50,42,51,52,39,28,36,10,8,12,30,9,11,29,13,27,26,20,19,18]
nearby_points = []
a=0
x70, y70 = None, None  # Initialize the position of the moving marker with ID 70
aruco_pos = {}
shortest_path = []
angles = []
########################################################################################
#####################functions############
def replace_coordinates_with_ids(pixel_coordinates_list, id_coordinates_dict):
    # Create a reverse mapping from coordinates to IDs for efficient lookup
    coordinates_to_id = {tuple(coord): id_ for id_, coord in id_coordinates_dict.items()}

    # Replace pixel coordinates with corresponding IDs
    replaced_ids_list = [coordinates_to_id.get(tuple(coord), None) for coord in pixel_coordinates_list]

    return replaced_ids_list
def calculate_distance(marker1, marker2):
    return np.sqrt((marker2[0] - marker1[0])**2 + (marker2[1] - marker1[1])**2)
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
        processed_data_dict = process_csv(csv_file_path)
        lat, lon = processed_data_dict[ids]
        print([ids, lat,lon])
        coordinate = [float(lat),float(lon)]
        write_csv(coordinate,"live_data.csv")
    except KeyError:
        pass 
################################################main sync code#######################################
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
def replace_coordinates_with_ids(pixel_coordinates_list, id_coordinates_dict):
    # Create a reverse mapping from coordinates to IDs for efficient lookup
    coordinates_to_id = {tuple(coord): id_ for id_, coord in id_coordinates_dict.items()}

    # Replace pixel coordinates with corresponding IDs
    replaced_ids_list = [coordinates_to_id.get(tuple(coord), None) for coord in pixel_coordinates_list]

    return replaced_ids_list
#############################CORNER DECTECTION FUNCTION ################
def detect_aruco_corner_coordinates(image, corner_index=0):
    # Load the image

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Define the ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    # Define parameters for ArUco detection
    parameters = aruco.DetectorParameters_create()

    # Detect ArUco markers in the image
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Initialize a dictionary to store corner coordinates
    corner_coordinates = {}

    # Print the coordinates of the specified corner for each detected ArUco marker
    if ids is not None:
        for i in range(len(ids)): 
            if len(corners[i][0]) > corner_index:  # Check if the specified corner exists
                x, y = corners[i][0][corner_index]
                corner_coordinates[ids[i][0]] = (int(x), int(y))
            else:
                print(f"ArUco ID {ids[i][0]} does not have corner {corner_index+1}.")
    else:
        print("No ArUco markers detected.")
    
    return corner_coordinates

###########################BOT DECISION FUNCTIONS########################
def calc_angle(coord1, coord2):
    x2,y2 = coord1
    x1,y1 = coord2
    if (x2-x1)!=0 :
        return np.degrees(np.arctan((y2-y1)/(x2-x1))), (y2-y1), (x2-x1)
    else:
        return 90.0,(y2-y1), (x2-x1)

def angle(p1, p2, p3):
    """
    Calculate the angle formed by the vectors p1p2 and p2p3.
    """
    radians = math.atan2(p3[1] - p2[1], p3[0] - p2[0]) - math.atan2(p1[1] - p2[1], p1[0] - p2[0])
    return math.degrees(radians)
###########################SHORTEST PATH DUNCTIONS############
def calc_cen(coord1, coord2, xconstant, yconstant):
    # Calculate center point
    center_x = (coord1[0] + coord2[0]) / 2 + xconstant
    center_y = (coord1[1] + coord2[1]) / 2 + yconstant
    
    return (center_x, center_y)
class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = {}
        self.distances = {}

    def add_node(self, value):
        self.nodes.add(value)

    def add_edge(self, from_node, to_node, distance):
        self.edges.setdefault(from_node, []).append(to_node)
        self.edges.setdefault(to_node, []).append(from_node)
        self.distances[(from_node, to_node)] = distance
        self.distances[(to_node, from_node)] = distance

def heuristic(node, goal):
    dx = abs(node[0] - goal[0])
    dy = abs(node[1] - goal[1])
    return math.sqrt(dx**2 + dy**2)

def line_intersection(line1, line2):
    # Calculate the direction vectors
    p = line1[0]
    r = (line1[1][0] - line1[0][0], line1[1][1] - line1[0][1])
    q = line2[0]
    s = (line2[1][0] - line2[0][0], line2[1][1] - line2[0][1])
    
    # Calculate the denominator
    denominator = r[0]*s[1] - r[1]*s[0]

    # If the denominator is zero, the lines are parallel
    if denominator == 0:
        return None

    # Calculate the parameters
    t = ((q[0] - p[0])*s[1] - (q[1] - p[1])*s[0]) / denominator
    u = ((q[0] - p[0])*r[1] - (q[1] - p[1])*r[0]) / denominator

    # Check if the intersection point is within the line segments
    if 0 <= t <= 1 and 0 <= u <= 1:
        intersection_point = (p[0] + t*r[0], p[1] + t*r[1])
        return intersection_point
    else:
        return None

def astar(graph, start, goal, wall_lines):
    if start not in graph.nodes:
        print("Start node not in graph")
        return []

    frontier = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}

    while frontier:
        current_cost, current_node = heapq.heappop(frontier)

        if current_node == goal:
            break

        if current_node not in graph.edges:
            continue

        for next_node in graph.edges[current_node]:
            new_cost = cost_so_far[current_node] + graph.distances[(current_node, next_node)]

            # Check for intersection with wall lines
            intersect_wall = False
            for wall_line in wall_lines:
                intersection = line_intersection((current_node, next_node), wall_line)
                if intersection:
                    intersect_wall = True
                    break

            if not intersect_wall and (next_node not in cost_so_far or new_cost < cost_so_far[next_node]):
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(next_node, goal)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current_node

    path = []
    while current_node != start:
        path.append(current_node)
        current_node = came_from[current_node]
    path.append(start)
    path.reverse()

    return path
#for debugging
def visualize_points_with_walls(aruco_corners, wall_lines, path=None):
    plt.gca().invert_yaxis()  
    for corner in aruco_corners:
        plt.plot(corner[0], corner[1], 'o', markersize=5, color='red')  

    for line in wall_lines:
        plt.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], color='black')

    if path:
        for i in range(len(path) - 1):
            plt.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], color='green')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('ArUco Corner Points with Walls')
    plt.grid(True)
    plt.show()

def calculate_angles(path):
    angles = []
    orientation = []
    for i in range(len(path) - 1):
        angle, dy, dx = calc_angle(path[i], path[i + 1])
        if (dy<0 and (-90<angle<-45 or 90>angle>45)):
            orientation.append("up")
        elif (dy>0 and (-90<angle<-45 or 90>angle>45)):
            orientation.append("down")
        elif (dx>0 and -45<angle<45):
            orientation.append("right")
        elif (dx<0 and -45<angle<45):
            orientation.append("left")
        angles.append(angle)
    # print(orientation)
    return angles,orientation

def decision(orientation):
    dir = []
    dir.append("straight") if orientation[0]=="up" else dir.append("right")
    for i in range(len(orientation)-1):
        if orientation[i]==orientation[i+1]:
            dir.append("straight")
        
        elif orientation[i]=="up" :
            if orientation[i+1]=="right":
                dir.append("right")
            elif orientation[i+1]=="left":
                dir.append("left")
        
        elif orientation[i]=="down" :
            if orientation[i+1]=="right":
                dir.append("left")
            elif orientation[i+1]=="left":
                dir.append("right")

        elif orientation[i]=="right" :
            if orientation[i+1]=="up":
                dir.append("left")
            elif orientation[i+1]=="down":
                dir.append("right")
        
        elif orientation[i]=="left" :
            if orientation[i+1]=="up":
                dir.append("right")
            elif orientation[i+1]=="down":
                dir.append("left")
    
    return dir
def filter_points_by_angle(points, angles, deviation_ranges1=(-80, -100), deviation_ranges3=(80, 100), deviation_ranges2=(-10, 10)):
    filtered_points = [points[0]]  # Include the first point always

    # Initialize flags
    flag1 = False
    flag2 = False

    for i in range(1,len(angles)):
        # Check angle conditions for i-1, i and i, i+1

        if (angles[i-1] < deviation_ranges1[0] or angles[i-1] > deviation_ranges1[1]) and (angles[i-1] < deviation_ranges3[0] or angles[i-1] > deviation_ranges3[1]) and (angles[i-1] < deviation_ranges2[0] or angles[i-1] > deviation_ranges2[1]):
            filtered_points.append(points[i-1])
            filtered_points.append(points[i])
            break
        
        
        if (deviation_ranges1[0] <= angles[i] <= deviation_ranges1[1] or deviation_ranges3[0] <= angles[i] <= deviation_ranges3[1]):
            # flag1 = True
            if deviation_ranges2[0] <= angles[i-1] <= deviation_ranges2[1]:
                # flag2 = True
                filtered_points.append(points[i])
                filtered_points.append(points[i-1])
        
        if (deviation_ranges1[0] <= angles[i-1] <= deviation_ranges1[1] or deviation_ranges3[0] <= angles[i-1] <= deviation_ranges3[1]):
            # flag1 = True
            if deviation_ranges2[0] <= angles[i] <= deviation_ranges2[1]:
                # flag2 = True
                filtered_points.append(points[i-1])
                filtered_points.append(points[i])

    filtered_points.append(points[-1])  # Include the last point
    return filtered_points
###############################FUNCTION TO GET A FRAME FROM THE VIDEO##############
def map_frame():
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FPS, 60)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    duration = 3  # in seconds
    start_time = cv2.getTickCount()
    
    while True:
        ret, __ = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        elapsed_time = (cv2.getTickCount() - start_time) / cv2.getTickFrequency()
        if elapsed_time >= duration:
            break
    
    __,frame = cap.read()
    frame = cv2.imread("/home/pradhyumna/hardware_round/eyrc23_GG_1298/Task_5A/task_Integration/output_image.png")
    return frame 
##############################FUNCTIONS FOR THE EVENT THING###############

def sort_dict_by_array_order(my_dict, order_array):
    sorted_dict = dict(sorted(my_dict.items(), key=lambda item: order_array.index(item[1][0])))
    return sorted_dict

def classification(img):
    combat = "combat"
    rehab = "human_aid_rehabilitation"
    military_vehicles = "military_vehicles"
    fire = "fire"
    destroyed_building = "destroyed_buildings"

    # Load your pre-trained model from TensorFlow Hub or another source
    feature_extractor_model = "https://www.kaggle.com/models/google/mobilenet-v2/frameworks/TensorFlow2/variations/tf2-preview-classification/versions/4"
    pretrained_model_without_top_layer = hub.KerasLayer(
        feature_extractor_model, input_shape=(224, 224, 3), trainable=False)  # Adjust input shape for 50x50 images
    number_of_classes = 5

    # Build your model
    model = tf.keras.Sequential([
    pretrained_model_without_top_layer,  # Include your pretrained model without the top layer
    tf.keras.layers.Flatten(),  # Flatten the output of the pretrained model
    tf.keras.layers.Dense(256, activation='relu'),
    tf.keras.layers.Dropout(0.5),  # Add dropout for regularization
    tf.keras.layers.Dense(128, activation='relu'),
    tf.keras.layers.Dropout(0.5),  # Add dropout for regularization
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dropout(0.5),  # Add dropout for regularization
    tf.keras.layers.Dense(5, activation='softmax')  # Adjust the number of classes as needed
])

    model.load_weights('C:/Users/prit4/OneDrive/Desktop/stuff/active_Github_repos/eyrc23_GG_1298/Task_5/final/17-01.h5')

    # Resize the input image to 50x50 if needed
    
    # Normalize the image pixel values
    img = img / 255

    # Expand dimensions to match the input shape expected by the model
    img = np.expand_dims(img, axis=0)

    # Make predictions
    predictions = model.predict(img)

    # Find the predicted class index
    x = np.argmax(predictions, axis=1)

    if x == 0:
        return combat
    if x == 1:
        return destroyed_building
    if x == 2:
        return  fire
    if x == 3:
        return rehab
    else:
        return military_vehicles

def eventReturn(img):
    global detected_list_final
    # View the loaded variable
    # img = map_frame() 
    
    lower_range = np.array([200,186,190])
    upper_range = np.array([255, 255, 255])  # Upper range (B, G, R)

    # Create a mask based on the specified range
    mask = cv2.inRange(img, lower_range, upper_range)
    points = []


    # Apply the mask to the original image
    result = cv2.bitwise_and(img, img, mask=mask)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Approximate the contour to a polygon with less vertices
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Ignore small shapes by setting a minimum area threshold
        if cv2.contourArea(approx) > 1600:
            #cv.drawContours(img, [approx], 0, (0, 255, 0), 2)
            points.append(approx)

    for i,p in enumerate(points):
        if(i%2!=0):       #i%2!=0 and i<10                   
            # Extract the (x, y) coordinates from the list
            xy_coordinates = [coord[0] for coord in p]
            coordinates_array = np.array(xy_coordinates)
            # Find the bounding rectangle of the coordinates
            x, y, w, h = cv2.boundingRect(coordinates_array)

            # Crop the region of interest (ROI) using the bounding rectangle
            event_list.append(cv2.resize(img[y:y + h, x:x + w], (224, 224)))
            # print(int((i-1)/2),points[i][0])
            detected_list[event_list[int((i-1)/2)]] = [cv2.resize(img[y:y + h, x:x + w], (224, 224)),points[i]]

    detected_list.pop('_',None)

    for d,c in detected_list.items():    
        count = 0    

        lower_range1 = np.array([100,100,100])
        upper_range1 = np.array([170,170,175])
        
        mask_1 = cv2.inRange(c[0],lower_range1,upper_range1)
        contours, _ = cv2.findContours(mask_1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Approximate the contour to a polygon with less vertices
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Ignore small shapes by setting a minimum area threshold
            if cv2.contourArea(approx) > 30:
                count = count + 1

        if count>12:
            # c[0] = cv.fastNlMeansDenoisingColored(c[0], None, 10, 10, 7, 21)
            detected_list_final[d] = [c[0],c[1]] 
        

    for d,c in detected_list_final.items():
        imag,points = c 
        
        detected_list_final[d] = [classification(imag),points]
        
    detected_list_final = sort_dict_by_array_order(detected_list_final,prioriy_order) 
    # print(detected_list_final)

    for c,d in detected_list_final.items():
        for i in range(len(d[1])):
            d[1][i] = d[1][i].tolist() 
        detected_list_final[c] = d[1].tolist()
        p1,p2 = detected_list_final[c][0][0], detected_list_final[c][1][0]
        detected_list_final[c] = [(p1[0]+p2[0])/2,(p1[1]+p2[1])/2]

    return detected_list_final


############################ main part ############
graph = Graph()
# image = map_frame()
image = cv2.imread('C:/Users/prit4/Downloads/sample.jpg')
# aruco_corner_dict = detect_aruco_corner_coordinates(image, 0)
# aruco_corner_dict={7: (80, 551), 6: (617, 547), 21: (249, 489), 20: (295, 487), 19: (349, 485), 17: (467, 484), 18: (406, 484), 16: (517, 483), 15: (579, 480), 23: (101, 468), 14: (594, 438), 24: (101, 431), 13: (595, 400), 22: (101, 399), 25: (170, 388), 26: (230, 382), 27: (292, 378), 28: (396, 378), 29: (520, 376), 11: (591, 315), 9: (589, 282), 49: (89, 280), 30: (514, 270), 32: (377, 269), 31: (414, 267), 34: (240, 266), 33: (290, 267), 12: (591, 245), 50: (89, 218), 8: (591, 204), 36: (516, 188), 38: (429, 185), 37: (475, 187), 35: (379, 183), 39: (288, 183), 40: (245, 180), 41: (195, 178), 42: (152, 178), 51: (88, 153), 10: (594, 134), 43: (496, 96), 44: (450, 90), 45: (396, 90), 52: (85, 90), 46: (341, 88), 47: (290, 87), 48: (246, 87), 53: (103, 41), 54: (155, 28), 5: (57, 6)}
# aruco_corner_dict = {6: [880.75, 693.75], 7: [235.0, 689.5], 21: [439.0, 617.5], 17: [700.5, 615.5], 16: [761.0, 615.5], 19: [558.5, 615.5], 20: [494.0, 616.25], 18: [627.0, 614.75], 15: [835.5, 612.75], 23: [261.75, 591.0], 14: [853.75, 561.5], 24: [263.5, 547.5], 13: [855.0, 516.0], 22: [262.75, 507.25], 25: [346.25, 495.5], 26: [417.75, 490.0], 28: [616.0, 486.5], 29: [766.0, 486.5], 27: [492.0, 485.0], 11: [852.75, 415.75], 9: [849.25, 375.0], 49: [251.0, 364.75], 30: [759.25, 360.0], 32: [596.0, 357.25], 31: [641.0, 356.0], 33: [492.0, 353.25], 34: [432.0, 351.0], 12: [850.75, 330.75], 36: [762.5, 262.25], 37: [714.25, 260.5], 38: [658.5, 258.0], 35: [600.25, 256.5], 40: [439.0, 249.0], 41: [381.25, 247.0], 51: [253.0, 215.0], 10: [854.5, 199.5], 43: [739.25, 153.0], 44: [686.0, 146.5], 46: [555.25, 141.75], 47: [494.0, 140.0], 52: [251.0, 140.25], 48: [442.5, 139.75], 53: [274.0, 82.25], 4: [893.75, 36.0], 42: [328.25, 245.0], 5: [220.75, 41.0]}
aruco_corner_dict = {7: (106.75, 456.0), 6: (538.75, 455.0), 21: (244.5, 406.0), 20: (281.5, 405.0), 19: (324.5, 404.0), 16: (459.5, 403.5), 17: (419.25, 403.75), 18: (370.5, 403.5), 15: (508.5, 401.0), 23: (125.5, 389.5), 14: (520.75, 367.0), 24: (126.75, 360.5), 13: (521.75, 336.5), 22: (126.5, 333.5), 25: (182.5, 324.75), 26: (230.5, 320.5), 29: (463.0, 317.25), 28: (363.5, 317.5), 27: (280.5, 317.5), 11: (520.5, 269.5), 9: (518.5, 242.5), 49: (118.5, 238.5), 30: (458.5, 232.5), 32: (349.5, 231.5), 31: (380.0, 230.5), 33: (280.5, 228.5), 34: (240.25, 227.5), 12: (519.5, 213.5), 50: (120.0, 188.5), 8: (521.0, 181.5), 36: (460.5, 167.5), 37: (428.5, 166.5), 38: (391.5, 165.0), 35: (353.0, 164.25), 39: (279.5, 162.5), 40: (245.25, 159.5), 41: (206.5, 159.0), 42: (170.5, 157.5), 51: (119.75, 137.5), 10: (522.0, 125.5), 43: (445.25, 94.75), 44: (410.0, 90.75), 45: (366.0, 89.5), 46: (323.0, 87.5), 52: (118.5, 87.5), 47: (282.0, 86.75), 48: (247.0, 86.5), 53: (133.5, 48.5), 54: (175.25, 38.75), 5: (97.5, 20.5)}

aruco_corners =list(aruco_corner_dict.values())
# print(aruco_corner_dict)
wall_lines = [
    (calc_cen(aruco_corner_dict[24], aruco_corner_dict[25], 0, 0), calc_cen(aruco_corner_dict[27], aruco_corner_dict[20], 0, 0)),
    (calc_cen(aruco_corner_dict[42], aruco_corner_dict[25], -10, 50), calc_cen(aruco_corner_dict[27], aruco_corner_dict[33], 0, 0)),
    (calc_cen(aruco_corner_dict[42], aruco_corner_dict[25],0, -50), calc_cen(aruco_corner_dict[33], aruco_corner_dict[39], -10, 0)),
    (calc_cen(aruco_corner_dict[19], aruco_corner_dict[28], 0, 0), calc_cen(aruco_corner_dict[29], aruco_corner_dict[16], 0, 0)),
    (calc_cen(aruco_corner_dict[30], aruco_corner_dict[29], -10, 0), calc_cen(aruco_corner_dict[31], aruco_corner_dict[28], -30, 0)),
    (calc_cen(aruco_corner_dict[36], aruco_corner_dict[30], 0, 0), calc_cen(aruco_corner_dict[35], aruco_corner_dict[32], 5, 0)),
    (calc_cen(aruco_corner_dict[43], aruco_corner_dict[36], 0, 0), calc_cen(aruco_corner_dict[48], aruco_corner_dict[42], 0, 0)),
    (calc_cen(aruco_corner_dict[48], aruco_corner_dict[42], 0, 0), calc_cen(aruco_corner_dict[42], aruco_corner_dict[51], 0, 0)),
]

event_order = eventReturn(image)
# print(event_order)
# Add nodes and edges
start = 7
first_value = tuple(tuple(event_order.values())[0])
# print(first_value)
##########################redundant points#################
# aruco_corners.append(calc_cen(aruco_corner_dict[28], aruco_corner_dict[27], 0, 0))
# aruco_corners.append(calc_cen(aruco_corner_dict[22], aruco_corner_dict[25], 0, 0))
###########################the image points################
aruco_corners.append(first_value)
# print(aruco_corners)
start_node = aruco_corner_dict[start]
goal_node =first_value
# print(goal_node)
for aruco in aruco_corners:
    graph.add_node(aruco)
for i in range(len(aruco_corners)):
    for j in range(i + 1, len(aruco_corners)):
        distance = math.sqrt((aruco_corners[i][0] - aruco_corners[j][0])**2 + (aruco_corners[i][1] - aruco_corners[j][1])**2)
        if distance < 120:  # Adjust this threshold as needed
            graph.add_edge(aruco_corners[i], aruco_corners[j], distance)
if start_node not in graph.nodes:
    graph.add_node(start_node)
shortest_path = astar(graph, start_node, goal_node, wall_lines)
visualize_points_with_walls(aruco_corners, wall_lines, shortest_path)
angles, orie = calculate_angles(shortest_path)
filtered_points = filter_points_by_angle(shortest_path, angles)
filtered_angles= calculate_angles(filtered_points)

for id in nearby_aruco:
    nearby_points.append(aruco_corner_dict[id])
    # nearby_aruco.pop(len(nearby_aruco)-1)
# print(shortest_path)
# print(nearby_points)
for point in shortest_path:
    if(point in nearby_points):
        pass
    else:
        shortest_path.remove(point)

final_angles,orie = calculate_angles(shortest_path)
com_to_send= decision(orie)
print(com_to_send)
fil_id_list =replace_coordinates_with_ids(shortest_path,aruco_corner_dict)

marker_100_position = None
nearest_markers_history = []  # List to store history of nearest markers
req_id = []
count = 0
websocket_client()

