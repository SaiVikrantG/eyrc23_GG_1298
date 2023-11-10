import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

event_list = []

def event_identification(arena):        # NOTE: You can tweak this function in case you need to give more inputs 
    ''' 
    Purpose:
    ---
    This function will select the events on arena image and extract them as
    separate images.
    
    Input Arguments:
    ---
    `arena`: Image of arena detected by arena_image() 	
    
    Returns:
    ---
    `event_list`,  : [ List ]
                            event_list will store the extracted event images

    Example call:
    ---
    event_list = event_identification(arena)
    '''
    img =arena
    lower_range = np.array([255, 254, 254])  # Lower range (B, G, R)
    upper_range = np.array([255, 255, 255])  # Upper range (B, G, R)
    cropped_image=[]
    # Create a mask based on the specified range
    mask = cv.inRange(img, lower_range, upper_range)
    #view("mask",mask)
    points = []
    # Apply the mask to the original image
    result = cv.bitwise_and(img, img, mask=mask)
    #view("result",result)
    contours, _ = cv.findContours(
        mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # Approximate the contour to a polygon with less vertices
        epsilon = 0.04 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)

        # Ignore small shapes by setting a minimum area threshold
        if cv.contourArea(approx) > 1500:
            #cv.drawContours(img, [approx], 0, (0, 255, 0), 2)
            #view("image",img)
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
        

    
    return event_list


def arena_image(arena_path):            # NOTE: This function has already been done for you, don't make any changes in it.
    ''' 
    Purpose:
    ---
    This function will take the path of the generated image as input and 
    read the image specified by the path.
    
    Input Arguments:
    ---
    `arena_path`: Generated image path i.e. arena_path (declared above) 	
    
    Returns:
    ---
    `arena` : [ Numpy Array ]

    Example call:
    ---
    arena = arena_image(arena_path)
    '''
    '''
    ADD YOUR CODE HERE
    '''
    frame = cv.imread(arena_path)
    
    arena = cv.resize(frame, (700, 700))
    return arena 

def view(name,img):
    cv.imshow(name,img)
    if cv.waitKey(0) == ord('q'):
        cv.destroyAllWindows()
        return

def remove(img):
    # Read the image

    # Convert the image to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Threshold the image to create a binary mask of the white regions
    _, mask = cv.threshold(gray, 240, 255, cv.THRESH_BINARY)

    # Find contours in the mask
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Find the contour with the largest area (assuming it corresponds to the frame)
    largest_contour = max(contours, key=cv.contourArea)

    # Create a mask for the frame
    frame_mask = np.zeros_like(gray)
    cv.drawContours(frame_mask, [largest_contour], 0, 255, thickness=cv.FILLED)

    # Invert the frame mask to get a mask for the content
    content_mask = cv.bitwise_not(frame_mask)

    # Extract the region of interest (ROI) excluding the white frame
    result = cv.bitwise_and(img, img, mask=content_mask)

    view("result",cv.resize(img,(440,440)))


arena = arena_image("sample.png")
event_list = event_identification(arena)
print(len(event_list))
print(event_list[0].shape)

for i in event_list:
    remove(i)

