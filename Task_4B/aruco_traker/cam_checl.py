import cv2 
cap = cv2.VideoCapture(0)

while True:
    ret,frame = cap.read()

    if ret==False:
        break

    cv2.imshow("frame",frame)
    print(frame)

cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()

