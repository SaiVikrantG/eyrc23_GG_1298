import heapq
import matplotlib.pyplot as plt

# Define the maze layout as a grid (0 for empty space, 1 for walls)
maze = [
    [0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 1],
    [0, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0]
]
start_position = (0, 0)
goal_position = (6, 6)


# Define a function to visualize the maze
def visualize_maze(maze, start, goal, path=None):
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == 1:
                plt.plot(j, i, 's', markersize=20, color='black')  # Plot walls as squares
    plt.plot(start[1], start[0], 'o', markersize=10, color='green')  # Plot start position
    plt.plot(goal[1], goal[0], 'o', markersize=10, color='red')  # Plot goal position
    if path:
        for i in range(len(path) - 1):
            plt.plot([path[i][1], path[i + 1][1]], [path[i][0], path[i + 1][0]], color='blue')  # Plot path
    plt.xlim(-0.5, len(maze[0]) - 0.5)
    plt.ylim(-0.5, len(maze) - 0.5)
    plt.gca().invert_yaxis()  # Invert y-axis to display maze properly
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('A* Shortest Path in Maze')
    plt.grid(True)
    plt.show()

# Define the A* algorithm
def astar(maze, start, goal):
    frontier = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}

    while frontier:
        current_cost, current_node = heapq.heappop(frontier)

        if current_node == goal:
            break

        for next_node in get_neighbors(maze, current_node):
            new_cost = cost_so_far[current_node] + 1
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(next_node, goal)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current_node

    path = []
    current_node = goal
    while current_node != start:
        path.append(current_node)
        current_node = came_from[current_node]
    path.append(start)
    path.reverse()
    return path

# Define a function to get neighboring nodes
def get_neighbors(maze, node):
    neighbors = []
    row, col = node
    if row > 0 and maze[row - 1][col] == 0:
        neighbors.append((row - 1, col))
    if row < len(maze) - 1 and maze[row + 1][col] == 0:
        neighbors.append((row + 1, col))
    if col > 0 and maze[row][col - 1] == 0:
        neighbors.append((row, col - 1))
    if col < len(maze[0]) - 1 and maze[row][col + 1] == 0:
        neighbors.append((row, col + 1))
    return neighbors

# Define a heuristic function (Manhattan distance)
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# Find the shortest path using A*
shortest_path = astar(maze, start_position, goal_position)
print("Shortest Path:", shortest_path)

# Visualize the maze and the shortest path
visualize_maze(maze, start_position, goal_position, shortest_path)
