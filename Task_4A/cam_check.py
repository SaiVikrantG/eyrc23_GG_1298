import cv2 
import numpy as np 
cap = cv2.VideoCapture(3)

while True:
    ret,frame = cap.read()
    lower_range = np.array([190, 186, 180])  # Lower range (B, G, R)
    upper_range = np.array([255, 255, 255])  # Upper range (B, G, R)

    #frame = rotate_image(frame,90)
    mask = cv2.inRange(frame, lower_range, upper_range)

    # Apply the mask to the original frame
    result = cv2.bitwise_and(frame, frame, mask=mask)
    x1,y1 = 152,25 
    x2,y2 = 610,478
    # Crop the image using NumPy array slicing
    #frame = frame[y1:y2, x1:x2]
    cv2.imshow("Mask",mask)
    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()
        exit()
