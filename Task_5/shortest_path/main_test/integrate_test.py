import heapq
import math
import matplotlib.pyplot as plt
###########################FUNCTIONS########################
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

ac = {6: (1018, 666), 7: (399, 639), 16: (903, 586), 15: (975, 585), 17: (844, 583), 18: (774, 580), 19: (708, 578), 20: (647, 576), 21: (595, 576), 23: (429, 546), 14: (994, 535), 24: (433, 505), 13: (997, 491), 22: (434, 466), 29: (911, 460), 25: (513, 457), 28: (766, 455), 26: (580, 454), 27: (650, 451), 11: (996, 395), 9: (994, 355), 30: (907, 339), 49: (430, 333), 31: (795, 332), 32: (751, 332), 33: (654, 326), 34: (598, 324), 12: (996, 314), 8: (999, 268), 50: (434, 265), 36: (912, 246), 37: (866, 243), 38: (814, 240), 35: (759, 238), 39: (656, 233), 40: (608, 229), 41: (555, 226), 42: (506, 223), 51: (437, 194), 10: (1002, 189), 43: (892, 143), 44: (842, 137), 45: (780, 134), 46: (720, 130), 47: (662, 129), 48: (614, 128), 52: (437, 126), 53: (460, 73), 54: (518, 61), 4: (1041, 37), 5: (413, 35)}
aruco_corners =list(ac.values())
wall_lines = [
    (calc_cen(ac[24], ac[25], 0, 0), calc_cen(ac[27], ac[20], 0, 0)),
    (calc_cen(ac[42], ac[25], -10, 50), calc_cen(ac[27], ac[33], 0, 0)),
    (calc_cen(ac[42], ac[25], -10, -50), calc_cen(ac[33], ac[39], 10, 0)),
    (calc_cen(ac[19], ac[28], 0, 0), calc_cen(ac[29], ac[16], 0, 0)),
    (calc_cen(ac[30], ac[29], 0, 0), calc_cen(ac[31], ac[28], -30, 0)),
    (calc_cen(ac[36], ac[30], 0, 0), calc_cen(ac[33], ac[32], 35, -30)),
    (calc_cen(ac[43], ac[36], 0, 0), calc_cen(ac[48], ac[42], 0, 0)),
    (calc_cen(ac[48], ac[42], 0, 0), calc_cen(ac[42], ac[51], 0, 0)),
]
aruco_corners.append(calc_cen(ac[52],ac[48],0,0))
aruco_corners.append(calc_cen(ac[34],ac[49],0,0))
aruco_corners.append(calc_cen(ac[31],ac[30],0,0))
aruco_corners.append(calc_cen(ac[28],ac[29],0,0))
aruco_corners.append(calc_cen(ac[21],ac[23],0,0))
graph = Graph()

# Add nodes and edges
for aruco in aruco_corners:
    graph.add_node(aruco)
for i in range(len(aruco_corners)):
    for j in range(i + 1, len(aruco_corners)):
        distance = math.sqrt((aruco_corners[i][0] - aruco_corners[j][0])**2 + (aruco_corners[i][1] - aruco_corners[j][1])**2)
        if distance < 135:  # Adjust this threshold as needed
            graph.add_edge(aruco_corners[i], aruco_corners[j], distance)

# Ensure that the starting node is added to the graph
start_node = ac[30]
goal_node = ac[29]
if start_node not in graph.nodes:
    graph.add_node(start_node)

shortest_path = astar(graph, start_node, goal_node, wall_lines)
print("Shortest Path:", shortest_path)

visualize_points_with_walls(aruco_corners, wall_lines, shortest_path)
