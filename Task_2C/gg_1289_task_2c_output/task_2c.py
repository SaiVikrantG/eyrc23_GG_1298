'''
*****************************************************************************************
*
*        		===============================================
*           		GeoGuide(GG) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2C of GeoGuide(GG) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
############################## FILL THE MANDATORY INFORMATION BELOW ###############################

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_2c.py
# Functions:	    [`classify_event(image)` ]
###################################################################################################

# IMPORTS (DO NOT CHANGE/REMOVE THESE IMPORTS)
from sys import platform
import numpy as np
import subprocess
import cv2 as cv       # OpenCV Library
import shutil
import ast
import sys
import os

# Additional Imports
from tensorflow.keras.models import load_model
import tensorflow_hub as hub
import tensorflow as tf
import cv2
from tensorflow.keras.layers import Conv2D,MaxPool2D,Dropout,concatenate, ReLU, AvgPool2D
from tensorflow.keras.layers import Input
from tensorflow.keras.models import Model

'''
You can import your required libraries here
'''

# DECLARING VARIABLES (DO NOT CHANGE/REMOVE THESE VARIABLES)
arena_path = "arena.png"            # Path of generated arena image
event_list = []
detected_list = []

# Declaring Variables
'''
You can delare the necessary variables here
'''

# EVENT NAMES
'''
We have already specified the event names that you should train your model with.
DO NOT CHANGE THE BELOW EVENT NAMES IN ANY CASE
If you have accidently created a different name for the event, you can create another 
function to use the below shared event names wherever your event names are used.
(Remember, the 'classify_event()' should always return the predefined event names)  
'''
combat = "combat"
rehab = "humanitarianaid"
military_vehicles = "militaryvehicles"
fire = "fire"
destroyed_building = "destroyedbuilding"

# Extracting Events from Arena
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
        
    
    return event_list

# Event Detection
def classify_event(image):
    ''' 
    Purpose:
    ---
    This function will load your trained model and classify the event from an image which is 
    sent as an input.
    
    Input Arguments:
    ---
    `image`: Image path sent by input file 	
    
    Returns:
    ---
    `event` : [ String ]
                          Detected event is returned in the form of a string

    Example call:
    ---
    event = classify_event(image_path)
    '''
    '''
    ADD YOUR CODE HERE
    '''
    '''feature_extractor_model = "https://tfhub.dev/google/tf2-preview/mobilenet_v2/feature_vector/4"
    pretrained_model_without_top_layer = hub.KerasLayer(
    feature_extractor_model, input_shape=(224, 224, 3), trainable=False)
    number_of_classes = 5

    model = tf.keras.Sequential([
    pretrained_model_without_top_layer,
    tf.keras.layers.Dense(number_of_classes)])'''

    def fire_module(x,s1,e1,e3):
        s1x = Conv2D(s1,kernel_size = 1, padding = 'same')(x)
        s1x = ReLU()(s1x)
        e1x = Conv2D(e1,kernel_size = 1, padding = 'same')(s1x)
        e3x = Conv2D(e3,kernel_size = 3, padding = 'same')(s1x)
        x = concatenate([e1x,e3x])
        x = ReLU()(x)
        return x

    def SqueezeNet(input_shape, nclasses):
        input = Input(input_shape)
        x = Conv2D(96,kernel_size=(7,7),strides=(2,2),padding='same')(input)
        x = MaxPool2D(pool_size=(3,3), strides = (2,2))(x)
        x = fire_module(x, s1 = 16, e1 = 64, e3 = 64) #2
        x = fire_module(x, s1 = 16, e1 = 64, e3 = 64) #3
        x = fire_module(x, s1 = 32, e1 = 128, e3 = 128) #4
        x = MaxPool2D(pool_size=(3,3), strides = (2,2))(x)
        x = fire_module(x, s1 = 32, e1 = 128, e3 = 128) #5
        x = fire_module(x, s1 = 48, e1 = 192, e3 = 192) #6
        x = fire_module(x, s1 = 48, e1 = 192, e3 = 192) #7
        x = fire_module(x, s1 = 64, e1 = 256, e3 = 256) #8
        x = MaxPool2D(pool_size=(3,3), strides = (2,2))(x)
        x = fire_module(x, s1 = 64, e1 = 256, e3 = 256) #9
        x = Dropout(0.5)(x)
        x = Conv2D(nclasses,kernel_size = 1)(x)
        output = AvgPool2D(pool_size=(13,13))(x)
        model = Model(input, output)
        return model

    model = SqueezeNet((224,224,3),5)

    model.load_weights('my_model_weights.h5')
    img = image
    img = cv2.resize(img, (224, 224))
    img = img/255

    predictions = model.predict(np.expand_dims(img, axis=0))


    x = np.argmax(predictions, axis=1)

    if x == 0:
        return combat
    if x == 1:
        return destroyed_building
    if x == 2:
        return  fire
    if x == 3:
        return rehab
    else:
        return military_vehicles   


# ADDITIONAL FUNCTIONS
'''
Although not required but if there are any additonal functions that you're using, you shall add them here. 
'''
###################################################################################################
########################### DO NOT MAKE ANY CHANGES IN THE SCRIPT BELOW ###########################
def classification(event_list):
    for img_index in range(0,5):
        img = event_list[img_index]
        detected_event = classify_event(img)
        print((img_index + 1), detected_event)
        if detected_event == combat:
            detected_list.append("combat")
        if detected_event == rehab:
            detected_list.append("rehab")
        if detected_event == military_vehicles:
            detected_list.append("militaryvehicles")
        if detected_event == fire:
            detected_list.append("fire")
        if detected_event == destroyed_building:
            detected_list.append("destroyedbuilding")
    os.remove('arena.png')
    return detected_list

def detected_list_processing(detected_list):
    try:
        detected_events = open("detected_events.txt", "w")
        detected_events.writelines(str(detected_list))
    except Exception as e:
        print("Error: ", e)

def input_function():
    if platform == "win32":
        try:
            subprocess.run("input.exe")
        except Exception as e:
            print("Error: ", e)
    if platform == "linux":
        try:
            subprocess.run("./input")
        except Exception as e:
            print("Error: ", e)

def output_function():
    if platform == "win32":
        try:
            subprocess.run("output.exe")
        except Exception as e:
            print("Error: ", e)
    if platform == "linux":
        try:
            subprocess.run("./output")
        except Exception as e:
            print("Error: ", e)

###################################################################################################
def main():
    ##### Input #####
    input_function()
    #################

    ##### Process #####
    arena = arena_image(arena_path)
    event_list = event_identification(arena)
    detected_list = classification(event_list)
    detected_list_processing(detected_list)
    ###################

    ##### Output #####
    output_function()
    ##################

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted')
        if os.path.exists('arena.png'):
            os.remove('arena.png')
        if os.path.exists('detected_events.txt'):
            os.remove('detected_events.txt')
        sys.exit()
