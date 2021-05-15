# blob_detection.py

import cv2

cap = cv2.VideoCapture(0)  # Open the first camera connected to the computer.

# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Set thresholds for the image binarization.
params.minThreshold = 10
params.maxThreshold = 200
params.thresholdStep = 10

# Filter by colour.
params.filterByColor = False
params.blobColor = 200

# Filter by Area.
params.filterByArea = True
params.minArea = 15
params.maxArea = 2000

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.75

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.8

detector = cv2.SimpleBlobDetector_create(params)

while True:
    ret, frame = cap.read()  # Read an image from the frame.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    keypoints = detector.detect(gray)
    detected = cv2.drawKeypoints(frame, keypoints, None, (0,0,255),
                                 cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('detected', detected)  # Show the image on the display.
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Close the script when q is pressed.
        break

# Release the camera device and close the GUI.
cap.release()
cv2.destroyAllWindows()
