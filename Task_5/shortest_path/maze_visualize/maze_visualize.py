import matplotlib.pyplot as plt

# Function to visualize the ArUco corner points
def visualize_points(aruco_corners):
    plt.gca().invert_yaxis()  # Invert y-axis
    for corner in aruco_corners:
        plt.plot(corner[0], corner[1], 'o', markersize=5, color='red')  # Plot ArUco corner points
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('ArUco Corner Points')
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

# Visualize the ArUco corner points
visualize_points(aruco_corners)
