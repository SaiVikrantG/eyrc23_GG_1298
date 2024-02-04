import cv2
import cv2.aruco as aruco

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

# Replace 'your_image.jpg' with the new path to your image
# image_path = 'C:/Users/prit4/OneDrive/Desktop/stuff/active_Github_repos/eyrc23_GG_1298/Task_5/aruco_detectiom/sample2.jpg'
image_path = '/home/zero/Documents/active_git_repos/eyrc23_GG_1298/Task_5/aruco_detectiom/sample5.jpg'
# Replace 'corner_index' with the index of the corner you want to print (0 to 3)
corner_index = 0

# Call the function to detect ArUco corner coordinates
aruco_corner_coordinates = detect_aruco_corner_coordinates(image_path, corner_index)

# Print the dictionary containing ArUco marker corner coordinates
print(aruco_corner_coordinates)
pixel_locations_list = list(aruco_corner_coordinates.values())

# Printing the list of pixel locations
# print(pixel_locations_list)
