#!python3.8

from djitellopy import tello
import numpy as np
import cv2

me = tello.Tello()
me.connect()

CHECKERBOARD_ROW = 6
CHECKERBOARD_COL = 9

# Termination criteria for cornerSubPix
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((CHECKERBOARD_ROW*CHECKERBOARD_COL,3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD_COL,0:CHECKERBOARD_ROW].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space.
imgpoints = [] # 2d points in image plane.


print(me.get_battery())
me.streamon()


while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (960, 720))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (CHECKERBOARD_COL, CHECKERBOARD_ROW), None)  # Find the chess board corners.


    # If found, add object points, image points (after refining them).
    if ret == True:
        objpoints.append(objp)

        refined_corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(refined_corners)

        # Draw and display the corners.
        img = cv2.drawChessboardCorners(img, (CHECKERBOARD_COL, CHECKERBOARD_ROW), refined_corners, ret)
        cv2.imshow('Camera Calibration', img)
        # Delay for half a second so we don't end up with a massive number of points to process later.
        cv2.waitKey(500)
    else:
        cv2.imshow('Camera Calibration', img)



    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break



print("Calculating Camera Distortion...")
print("Depending on how many frames were captured, this may take some time...")
ret, camera_matrix, distortion_coeff, rvec, tvec = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save the camera matrix and distortion coefficients for later.
np.savetxt('src/config/camera_matrix.npy', camera_matrix)
np.savetxt('src/config/distortion_coeff.npy', distortion_coeff)

me.end()
cv2.destroyAllWindows()

