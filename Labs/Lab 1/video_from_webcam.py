# video_from_webcam.py

import cv2

cap = cv2.VideoCapture(0)  # Open the first camera connected to the computer.

while True:
    ret, frame = cap.read()  # Read an image from the frame.
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame', frame)  # Show the image on the display.
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Close the script when q is pressed.
        break

# Release the camera device and close the GUI.
cap.release()
cv2.destroyAllWindows()
