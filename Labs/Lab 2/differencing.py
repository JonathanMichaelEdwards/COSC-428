# differencing.py

import numpy as np
import cv2

def difference():
    cap = cv2.VideoCapture(-1)  # Open the webcam device.

    # Load two initial images from the webcam to begin.
    ret, img0 = cap.read()
    ret, img1 = cap.read()

    while True:
        diff = cv2.subtract(img0, img1)  # Calculate the differences of the two images.

        # Move the data in img0 to img1. Uncomment this line for differencing from the first frame.
        img1 = img0
        ret, img0 = cap.read()  # Grab a new frame from the camera for img0.

        cv2.imshow('Difference', diff)  # Display the difference to the screen.

        # Close the script when q is pressed.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def double_difference():
    cap = cv2.VideoCapture(-1)  # Open the webcam device.

    # Load three initial images from the webcam to begin.
    ret, img0 = cap.read()
    ret, img1 = cap.read()
    ret, img2 = cap.read()

    while True:
        diff01 = cv2.subtract(img0, img1)  # Calculate the differences of the two images.
        diff12 = cv2.subtract(img1, img2)  # Calculate the differences of the two images.

        double_diff = cv2.bitwise_and(diff01, diff12)

        img2 = img1  # Move the data in img1 to img2.
        img1 = img0  # Move the data in img0 to img1.
        ret, img0 = cap.read()

        ret, diff = cv2.threshold(diff, 24, 255, cv2.THRESH_BINARY)
        cv2.imshow('Double Difference', double_diff)  # Display the difference to the screen.
        
        # Close the script when q is pressed.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def difference_with_centroid():
    cap = cv2.VideoCapture(-1)  # Open the webcam device.

    # Load two initial images from the webcam to begin.
    ret, img0 = cap.read()
    ret, img1 = cap.read()

    while True:
        # Calculate the differences of the two images.
        diff = cv2.subtract(cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY),
                                            cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY))
        diff = cv2.medianBlur(diff,3)
        ret, diff = cv2.threshold(diff, 60, 255, cv2.THRESH_BINARY)

        # Move the data in img0 to img1. Uncomment this line for differencing from the first frame.
        img1 = img0
        ret, img0 = cap.read()  # Grab a new frame from the camera for img0.

        # Use the moments of the difference image to draw the centroid of the difference image.
        moments = cv2.moments(diff)
        if moments["m00"] != 0:  # Check for divide by zero errors.
            cX = int(moments["m10"] / moments["m00"])
            cY = int(moments["m01"] / moments["m00"])
            cv2.circle(diff, (cX, cY), 7, (255, 255, 255), -1)

        cv2.imshow('Difference', diff)  # Display the difference to the screen.

        # Close the script when q is pressed.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    difference()
    # double_difference()
    # difference_with_centroid()
