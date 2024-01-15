import cv2
import numpy as np

# Open the default camera (camera index 0)
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Define color range for mask
lower_range = np.array([255, 254, 254])  # Lower range (B, G, R)
upper_range = np.array([255, 255, 255])  # Upper range (B, G, R)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Break the loop if the camera feed has ended
    if not ret:
        break

    # Create a mask based on the specified range
    mask = cv2.inRange(frame, lower_range, upper_range)

    # Apply the mask to the original frame
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Find contours in the mask
    contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Process contours
    for contour in contours:
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if cv2.contourArea(approx) > 3500:
            x, y, w, h = cv2.boundingRect(approx)

            # Crop the region of interest (ROI) using the bounding rectangle
            cropped_image = frame[y:y + h, x:x + w]

            # Display the cropped image
            cv2.imshow("Cropped Image", cropped_image)

    # Display the original frame
    cv2.imshow("Original Frame", frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera object
cap.release()

# Destroy all OpenCV windows
cv2.destroyAllWindows()
