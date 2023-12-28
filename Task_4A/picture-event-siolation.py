import cv2 as cv
import numpy as np 
event_list = []

img =cv.imread("pics23-12/WIN_20231223_16_22_15_Pro.jpg")
lower_range = np.array([190, 186, 184])  # Lower range (B, G, R)
upper_range = np.array([255, 255, 255])  # Upper range (B, G, R)
cropped_image=[]
# Create a mask based on the specified range
mask = cv.inRange(img, lower_range, upper_range)
cv.imshow("mask",mask)
cv.imshow("Map",img)
points = []
# Apply the mask to the original image
result = cv.bitwise_and(img, img, mask=mask)
contours, _ = cv.findContours(
    mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
for contour in contours:
    # Approximate the contour to a polygon with less vertices
    epsilon = 0.04 * cv.arcLength(contour, True)
    approx = cv.approxPolyDP(contour, epsilon, True)

    # Ignore small shapes by setting a minimum area threshold
    if cv.contourArea(approx) > 1500:
        # cv.drawContours(img, [approx], 0, (0, 255, 0), 2)
        points.append(approx)
for i,p in enumerate(points):
    if(i%2!=0):
        # Extract the (x, y) coordinates from the list
        xy_coordinates = [coord[0] for coord in p]
        coordinates_array = np.array(xy_coordinates)
        # Find the bounding rectangle of the coordinates
        x, y, w, h = cv.boundingRect(coordinates_array)

        # Crop the region of interest (ROI) using the bounding rectangle
        event_list.append(cv.resize(img[y:y + h, x:x + w], (50, 50)))
# print(len(event_list))
# for c in event_list:
#     cv2.imshow("Original Image", cv.resize(c,(224,224)))
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
print(len(event_list))
for img in event_list:
    cv.imshow("Image event",img)
    
    if(cv.waitKey(0)==ord('q')):
        cv.destroyAllWindows()