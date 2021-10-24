# load_camera_calibration.py

import numpy as np
import cv2

cap = cv2.VideoCapture(-1)  # Open the first camera connected to the computer.

# Make sure we have the camera calibration files that we are expecting.
try:
    camera_matrix = np.loadtxt('./camera_matrix.npy')
    distortion_coeff = np.loadtxt('./distortion_coeff.npy')
    ret, frame = cap.read()
    h, w = frame.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeff, (w,h), 1, (w,h))
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, distortion_coeff, None, new_camera_matrix, (w,h), cv2.CV_16SC2)
except:
    raise FileExistsError("Missing Camera Matrices and Distortion Files.")

while True:
    ret, frame = cap.read()

    dst = cv2.undistort(frame, camera_matrix, distortion_coeff, None, new_camera_matrix)  # Undistort the image.

    # Crop the image.
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]

    cv2.imshow('Calibration Result', dst)  # Show the undistorted image to the screen.

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
