import cv2
 
cap = cv2.VideoCapture('./video/Final/paper.avi')

while cap.isOpened():
    ret, frame = cap.read() 

    if not ret:
        break

    if cv2.waitKey(100) & 0xFF == ord('q'):
        break


    h = frame.shape[0]
    w = frame.shape[1]
    
    # Draw axis for center point
    frame = cv2.line(frame, (0,int(h/2)), (w,int(h/2)), (0,0,255), 2) 
    frame = cv2.line(frame, (int(w/2),0), (int(w/2),h), (0,0,255), 2)

    cv2.imshow("Drones View", frame)


# Release the video file, and close the GUI.
cap.release()
cv2.destroyAllWindows()

