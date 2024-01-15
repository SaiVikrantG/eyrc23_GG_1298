import cv2

# Create a VideoCapture object
cap = cv2.VideoCapture(2)


# Loop to read and display video frames
while True:
    # Read a frame from the video
    ret, frame = cap.read()

    # Check if the video has ended
    if not ret:
        break

    # Display the frame
    cv2.imshow('arena live', frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(0) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close the OpenCV window
cap.release()
cv2.destroyAllWindows()
