import cv2 
import numpy as np

# Open the default camera (camera index 0)
cap = cv2.VideoCapture("afternoonFeed.mp4")

points = []
event_list = []
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Define color range for mask
lower_range = np.array([195, 190, 210])  # Lower range (B, G, R)
upper_range = np.array([255, 255, 255])  # Upper range (B, G, R)

while True:
    count = 0
    points.clear()
    event_list.clear()

    # Read a frame from the camera
    ret, frame = cap.read()

    # Break the loop if the camera feed has ended
    if not ret:
        break

    # Create a mask based on the specified range
    mask = cv2.inRange(frame, lower_range, upper_range)
    
    cv2.imshow("mask",mask)
    cv2.imshow("image",frame)

    # Apply the mask to the original frame
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Find contours in the mask
    contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Process contours
    for contour in contours:
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if cv2.contourArea(approx) > 4000:
            points.append(approx)
    
    for i, p in enumerate(points):
        if i == 5:
            break 

        # Extract the (x, y) coordinates from the list
        xy_coordinates = [coord[0] for coord in p]
        coordinates_array = np.array(xy_coordinates)
        
        # Find the bounding rectangle of the coordinates
        x, y, w, h = cv2.boundingRect(coordinates_array)

        # Crop the region of interest (ROI) using the bounding rectangle
        event_list.append(cv2.resize(frame[y:y + h, x:x + w], (50, 50)))

    for i in event_list:
        #gray_image = cv2.cvtColor(i, cv2.COLOR_BGR2GRAY)
        #output = clahe.apply(gray_image)
        cv2.imshow("Image"+str(count),i)
        count =count + 1
        if cv2.waitKey(0) == ord('q'):
            break

    
    

# Release the camera object
cap.release()

# Destroy all OpenCV windows
cv2.destroyAllWindows()
