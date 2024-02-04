import cv2
import cv2.aruco as aruco

def detect_aruco_corner_coordinates(image_path, corner_index=0):
    # Load the image
    image = cv2.imread(image_path)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Define the ArUco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

    # Define parameters for ArUco detection
    parameters = aruco.DetectorParameters_create()

    # Detect ArUco markers in the image
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Draw the detected markers on the image
    image_with_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)

    # Print the coordinates of the specified corner for each detected ArUco marker
    if ids is not None:
        for i in range(len(ids)):
            if len(corners[i][0]) > corner_index:  # Check if the specified corner exists
                x, y = corners[i][0][corner_index]
                # Draw the corner coordinates on the image
                cv2.putText(image_with_markers, f"({int(x)}, {int(y)})", (int(x), int(y)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
                # Print the coordinates on the console
                print(f"ArUco ID {ids[i][0]} - Corner {corner_index+1}: ({int(x)}, {int(y)})")
            else:
                print(f"ArUco ID {ids[i][0]} does not have corner {corner_index+1}.")

        # Display the image with markers and corner coordinates
        cv2.imshow('Detected ArUco Markers with Coordinates', image_with_markers)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No ArUco markers detected.")

# Replace 'your_image.jpg' with the new path to your image
# image_path = 'C:/Users/prit4/OneDrive/Desktop/stuff/active_Github_repos/eyrc23_GG_1298/Task_5/aruco_detectiom/sample2.jpg'
image_path = '/home/zero/Documents/active_git_repos/eyrc23_GG_1298/Task_5/aruco_detectiom/sample5.jpg'
# Replace 'corner_index' with the index of the corner you dwant to print (0 to 3)
corner_index = 0

detect_aruco_corner_coordinates(image_path, corner_index)
