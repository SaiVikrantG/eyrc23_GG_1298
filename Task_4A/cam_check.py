import cv2 
#import numpy as np 

cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture(0)
points = []
event_list = []

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

'''lower_range = np.array([190, 186, 254])  # Lower range (B, G, R)
upper_range = np.array([255, 255, 255])  # Upper range (B, G, R)'''

while True:
    ret, frame = cap.read()
    if not ret:
        print("No frame")
        break
    count = 0
    
    cv2.imshow("Video",frame)
    print(frame)

    '''# Create a mask based on the specified range
    mask = cv2.inRange(frame, lower_range, upper_range)

    # Apply the mask to the original frame
    result = cv2.bitwise_and(frame, frame, mask=mask)


    # Find contours in the mask
    contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)'''

    # Process contours
    '''for contour in contours:
        # Approximate the contour to a polygon with less vertices
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Ignore small shapes by setting a minimum area threshold
        if cv2.contourArea(approx) > 4000:
            # cv.drawContours(img, [approx], 0, (0, 255, 0), 2)
            points.append(approx)
    for i,p in enumerate(points):
        if(i==5):
            break
        # Extract the (x, y) coordinates from the list
        xy_coordinates = [coord[0] for coord in p]
        coordinates_array = np.array(xy_coordinates)
        # Find the bounding rectangle of the coordinates
        x, y, w, h = cv2.boundingRect(coordinates_array)

        # Crop the region of interest (ROI) using the bounding rectangle
        event_list.append(cv2.resize(frame[y:y + h, x:x + w], (50, 50)))'''

    # Display the original frame
    '''for i in event_list:
        count = count + 1
        cv2.imshow("Image"+str(count),i)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()'''
    
cv2.destroyAllWindows()
cap.release()

