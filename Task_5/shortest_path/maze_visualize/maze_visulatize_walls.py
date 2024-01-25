import matplotlib.pyplot as plt

# Function to visualize the ArUco corner points and walls
def visualize_points_with_walls(aruco_corners, wall_lines):
    plt.gca().invert_yaxis()  # Invert y-axis
    for corner in aruco_corners:
        plt.plot(corner[0], corner[1], 'o', markersize=5, color='red')  # Plot ArUco corner points

    # Plot wall lines
    for line in wall_lines:
        plt.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], color='black')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('ArUco Corner Points with Walls')
    plt.grid(True)
    plt.show()

# ArUco corner coordinates
aruco_corners = [
    (1018, 666), (399, 639), (903, 586), (975, 585), (844, 583),
    (774, 580), (708, 578), (647, 576), (595, 576), (429, 546),
    (994, 535), (433, 505), (997, 491), (434, 466), (911, 460),
    (513, 457), (766, 455), (580, 454), (650, 451), (996, 395),
    (994, 355), (907, 339), (430, 333), (795, 332), (751, 332),
    (654, 326), (598, 324), (996, 314), (999, 268), (434, 265),
    (912, 246), (866, 243), (814, 240), (759, 238), (656, 233),
    (608, 229), (555, 226), (506, 223), (437, 194), (1002, 189),
    (892, 143), (842, 137), (780, 134), (720, 130), (662, 129),
    (614, 128), (437, 126), (460, 73), (518, 61), (1041, 37),
    (413, 35)
]

# Wall lines
wall_lines = [
    ((514, 280), (657, 280)),
    ((765, 295), (909, 298)),
    ((773, 379), (889, 378)),
    ((499, 390), (660, 398)),
    ((520, 502), (636, 511)),
    ((775, 495), (890, 511)),
    ((531, 169), (891, 179)),
    ((531, 169), (450, 218)),
    ((500,500),  (456,487))
]

# Visualize the ArUco corner points with walls
visualize_points_with_walls(aruco_corners, wall_lines)
