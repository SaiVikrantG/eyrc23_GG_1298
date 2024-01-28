import matplotlib.pyplot as plt

def calc_cen(coord1, coord2, xconstant, yconstant):
    # Calculate center point
    center_x = (coord1[0] + coord2[0]) / 2 + xconstant
    center_y = (coord1[1] + coord2[1]) / 2 + yconstant
    
    return (center_x, center_y)

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

ac = {6: (1018, 666), 7: (399, 639), 16: (903, 586), 15: (975, 585), 17: (844, 583), 18: (774, 580), 19: (708, 578), 20: (647, 576), 21: (595, 576), 23: (429, 546), 14: (994, 535), 24: (433, 505), 13: (997, 491), 22: (434, 466), 29: (911, 460), 25: (513, 457), 28: (766, 455), 26: (580, 454), 27: (650, 451), 11: (996, 395), 9: (994, 355), 30: (907, 339), 49: (430, 333), 31: (795, 332), 32: (751, 332), 33: (654, 326), 34: (598, 324), 12: (996, 314), 8: (999, 268), 50: (434, 265), 36: (912, 246), 37: (866, 243), 38: (814, 240), 35: (759, 238), 39: (656, 233), 40: (608, 229), 41: (555, 226), 42: (506, 223), 51: (437, 194), 10: (1002, 189), 43: (892, 143), 44: (842, 137), 45: (780, 134), 46: (720, 130), 47: (662, 129), 48: (614, 128), 52: (437, 126), 53: (460, 73), 54: (518, 61), 4: (1041, 37), 5: (413, 35)}

# Corrected function call
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

visualize_points_with_walls(list(ac.values()), wall_lines)
