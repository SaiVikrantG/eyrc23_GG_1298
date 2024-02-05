import cv2
import cv2.aruco as aruco
import numpy as np
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
    # if (x2-x1)!=0 :
    return np.degrees(np.arctan((y2-y1)/(x2-x1))), (y2-y1), (x2-x1)


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

###################################VARIABLES#####################
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

a=0
x70, y70 = None, None  # Initialize the position of the moving marker with ID 70
aruco_pos = {}
shortest_path = []
angles = []
image_path = 'C:/Users/prit4/Downloads/sample.jpg'
# image_path='/home/zero/Documents/active_git_repos/eyrc23_GG_1298/Task_5/aruco_detectiom/sample5.jpg'
aruco_corner_dict = detect_aruco_corner_coordinates(image_path, 0)


image = cv2.imread(image_path)
aruco_corners =list(aruco_corner_dict.values())
aruco_corners.append(calc_cen(aruco_corner_dict[28], aruco_corner_dict[27], 0, 0))
aruco_corners.append(calc_cen(aruco_corner_dict[22], aruco_corner_dict[25], 0, 0))
aruco_corners.append(calc_cen(aruco_corner_dict[28], aruco_corner_dict[29], 0, 0))#image one
aruco_corners.append(calc_cen(aruco_corner_dict[49], aruco_corner_dict[34], 0, 0))#image one
corners, ids, _ = aruco.detectMarkers(image, aruco_dict)
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

graph = Graph()

# Add nodes and edges
for aruco in aruco_corners:
    graph.add_node(aruco)
for i in range(len(aruco_corners)):
    for j in range(i + 1, len(aruco_corners)):
        distance = math.sqrt((aruco_corners[i][0] - aruco_corners[j][0])**2 + (aruco_corners[i][1] - aruco_corners[j][1])**2)
        if distance < 120:  # Adjust this threshold as needed
            graph.add_edge(aruco_corners[i], aruco_corners[j], distance)
start_node = aruco_corner_dict[7]
goal_node = aruco_corner_dict[54]
if start_node not in graph.nodes:
    graph.add_node(start_node)

shortest_path = astar(graph, start_node, goal_node, wall_lines)
print("Shortest Path:", shortest_path)

visualize_points_with_walls(aruco_corners, wall_lines, shortest_path)
