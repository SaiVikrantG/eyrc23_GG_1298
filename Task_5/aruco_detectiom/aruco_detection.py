import cv2
import cv2.aruco as aruco

def detect_aruco_ids(image_path):
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
    
    # Display the image with markers
    cv2.imshow('Detected ArUco Markers', image_with_markers)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Print the detected ArUco IDs
    if ids is not None:
        print("Detected ArUco IDs:", ids.flatten())
    else:
        print("No ArUco markers detected.")

# Replace 'your_image.png' with the path to your .png image
image_path = 'C:/Users/prit4/OneDrive/Desktop/stuff/active_Github_repos/eyrc23_GG_1298/Task_5/aruco_detectiom/sample2.jpg'
detect_aruco_ids(image_path)
