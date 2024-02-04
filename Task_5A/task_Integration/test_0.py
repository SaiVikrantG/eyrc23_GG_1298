import cv2
# import cv2 as aruco
import numpy as np
import time
import heapq
import math
import matplotlib.pyplot as plt
#############################CORNER DECTECTION FUNCTION ################
def detect_aruco_corner_coordinates(image_path, corner_index=0):
    # Load the image
    image = cv2.imread(image_path)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Define the ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

    # Define parameters for ArUco detection
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Detect ArUco markers in the image
    # corners, ids, _ = cv2.detectMarkers(gray, aruco_dict, parameters=parameters)
    corners, ids, rejectedCandidates = detector.detectMarkers(gray)

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

def calc_angle(coord1, coord2):
    x2, y2 = coord2
    x1, y1 = coord1
    
    # Check if the difference in x-coordinates is zero
    if (x2 - x1) != 0:
        return np.degrees(np.arctan((y2 - y1) / (x2 - x1))), (y2 - y1), (x2 - x1)
    else:
        # Handle division by zero, return a large angle
        return 90.0, (y2 - y1), (x2 - x1)

# Rest of the code...

def angle(p1, p2, p3):
    """
    Calculate the angle formed by the vectors p1p2 and p2p3.
    """
    radians = math.atan2(p3[1] - p2[1], p3[0] - p2[0]) - math.atan2(p1[1] - p2[1], p1[0] - p2[0])
    return math.degrees(radians)

def signal(direction: str):
    if direction=="right":
        print("right turn")
    elif direction=="left":
        print("left turn")
    elif direction=="straight":
        print("straight")
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
    return dx+dy

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

###################################VARIABLES#####################
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

nearby_aruco = [7,21,24,25,22,49,50,42,51,52,39,28,36,10,8,12,30,9,11,29,13,27,26,20,19,18]
nearby_points = []

a=0
x70, y70 = None, None  # Initialize the position of the moving marker with ID 70
aruco_pos = {}
shortest_path = []
angles = []
image_path = '/home/pradhyumna/hardware_round/eyrc23_GG_1298/Task_5A/task_Integration/test2.jpeg'
aruco_corner_dict = detect_aruco_corner_coordinates(image_path, 0)

dir = {-90:"up",
         0:"right",
        90:"down",
       180:"left"}
image = cv2.imread(image_path)
aruco_corners =list(aruco_corner_dict.values())
corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict)
wall_lines = [
    (calc_cen(aruco_corner_dict[24], aruco_corner_dict[25], 0, 0), calc_cen(aruco_corner_dict[27], aruco_corner_dict[20], 0, 0)),
    (calc_cen(aruco_corner_dict[42], aruco_corner_dict[25], -10, 50), calc_cen(aruco_corner_dict[27], aruco_corner_dict[33], 0, 0)),
    (calc_cen(aruco_corner_dict[42], aruco_corner_dict[25], -10, -50), calc_cen(aruco_corner_dict[33], aruco_corner_dict[39], 10, 0)),
    (calc_cen(aruco_corner_dict[19], aruco_corner_dict[28], 0, 0), calc_cen(aruco_corner_dict[29], aruco_corner_dict[16], 0, 0)),
    (calc_cen(aruco_corner_dict[30], aruco_corner_dict[29], 0, 0), calc_cen(aruco_corner_dict[31], aruco_corner_dict[28], -30, 0)),
    (calc_cen(aruco_corner_dict[36], aruco_corner_dict[30], 0, 0), calc_cen(aruco_corner_dict[33], aruco_corner_dict[32], 35, -30)),
    (calc_cen(aruco_corner_dict[43], aruco_corner_dict[36], 0, 0), calc_cen(aruco_corner_dict[48], aruco_corner_dict[42], 0, 0)),
    (calc_cen(aruco_corner_dict[48], aruco_corner_dict[42], 0, 0), calc_cen(aruco_corner_dict[42], aruco_corner_dict[51], 0, 0)),
]

graph = Graph()

# Add nodes and edges
for aruco in aruco_corners:
    graph.add_node(aruco)
for i in range(len(aruco_corners)):
    for j in range(i + 1, len(aruco_corners)):
        distance = math.sqrt((aruco_corners[i][0] - aruco_corners[j][0])**2 + (aruco_corners[i][1] - aruco_corners[j][1])**2)
        if distance < 200:  # Adjust this threshold as needed
            graph.add_edge(aruco_corners[i], aruco_corners[j], distance)

print(len(aruco_corner_dict))
start = 48
end = 28
start_node = aruco_corner_dict[start]
goal_node = aruco_corner_dict[end]
nearby_aruco.append(end)
nearby_aruco.append(start)
if start_node not in graph.nodes:
    graph.add_node(start_node)

shortest_path = astar(graph, start_node, goal_node, wall_lines)
print(nearby_aruco)
print("Shortest Path:", shortest_path)
angles, orie = calculate_angles(shortest_path)
# Existing code...

# Function to filter points based on angle criteria
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

# Filter points based on angle criteria
filtered_points = filter_points_by_angle(shortest_path, angles)

# Display the filtered points
print("Filtered Points based on angle criteria:", filtered_points)

# # Display the angles
# print("Angles between consecutive points in the shortest path:", angles)
# Calculate angles for the provided filtered points in the shortened path
filtered_angles,orie = calculate_angles(filtered_points)

# Display the angles for the filtered points
print('angles :', angles)
print("Angles between consecutive points in the filtered path:", filtered_angles)


visualize_points_with_walls(aruco_corners, wall_lines, shortest_path)
# visualize_points_with_walls(aruco_corners, wall_lines, filtered_points)
# Existing code...

# Function to calculate angles between consecutive points in the shortest path


for id in nearby_aruco:
    nearby_points.append(aruco_corner_dict[id])
    # nearby_aruco.pop(len(nearby_aruco)-1)

# print(nearby_points)
print(len(shortest_path))
for point in shortest_path:
    if(point in nearby_points):
        pass
    else:
        shortest_path.remove(point)
print(len(shortest_path))
visualize_points_with_walls(aruco_corners, wall_lines, shortest_path)
final_angles,orie = calculate_angles(shortest_path)
print(final_angles)
print(orie)
dec = decision(orie)
print(dec)




