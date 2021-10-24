import time, cv2
import numpy as np
import cv2.aruco as aruco
from threading import Thread
from djitellopy import Tello
import cameraLoad

tello = Tello()

tello.connect()

# keepRecording = True
tello.streamon()
frame_read = tello.get_frame_read()

# def videoRecorder():
    # create a VideoWrite object, recoring to ./video.avi
height, width, _ = frame_read.frame.shape
video = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'XVID'), 20, (width, height))

    # while keepRecording:
    #     video.write(frame_read.frame)
    #     time.sleep(1 / 20)



# Drone camera matrix(s)
_CAMERA_MATRIX = cameraLoad.getCameraMat()
_DISTORTION_COEFF = cameraLoad.getDistortionMat()

CAMERA_MATRIX = np.array([ 
                            [_CAMERA_MATRIX[0], _CAMERA_MATRIX[1], _CAMERA_MATRIX[2]], 
                            [_CAMERA_MATRIX[3], _CAMERA_MATRIX[4], _CAMERA_MATRIX[5]],
                            [_CAMERA_MATRIX[6], _CAMERA_MATRIX[7], _CAMERA_MATRIX[8]]
])

DISTORTION_COEFF = np.array([
                            _DISTORTION_COEFF
])

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)  # Retrieve one of the fiducial dictionaries.

def generate_marker(marker_id=24, size=200):
    # By default, generate marker #24 (200 pixels in size) from the dictionary.
    marker = aruco.drawMarker(dictionary, marker_id, size)
    cv2.imwrite('fiducial_{}.png'.format(marker_id), marker)

def rotation_vector_to_euler(rotation_vector):
    #Convert an OpenCV-style rotation vector into Euler angles in degrees.
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    euler_angles, _, _, _, _, _ = cv2.RQDecomp3x3(rotation_matrix)
    return euler_angles


# we need to run the recorder in a seperate thread, otherwise blocking options
#  would prevent frames from getting added to the video
# recorder = Thread(target=videoRecorder)
# recorder.start()

# tello.takeoff()
# tello.move_up(100)
# tello.rotate_counter_clockwise(360)
# tello.land()
while True:
    frame = frame_read.frame  # Read a frame.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray, dictionary)  # Attempt to detect markers in the frame.
    if ids is not None:  # We have detected some markers.
        # Estimate the pose of the detected markers (assumes marker is 7cm wide).
        poses = aruco.estimatePoseSingleMarkers(corners, 0.035, CAMERA_MATRIX, DISTORTION_COEFF)  # Marker Area defined
        rvec, tvec = poses[0:2]

        try:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)  # Draw the detected markers onto the frame.
        except TypeError:
            pass
        
        # Draw the estimated position for the detected markers and print the details in the terminal.
        for i in range(len(ids)):
            x, y, z = np.squeeze(tvec[i])
            rx, ry, rz = rotation_vector_to_euler(np.squeeze(rvec[i]))
            # Draw a 3D axis for each marker position.
            try:
                frame = aruco.drawAxis(frame, CAMERA_MATRIX, DISTORTION_COEFF, rvec[i], tvec[i], 0.035)
            except TypeError:
                pass
            print("ID: {}\n\tTranslation Vector: x: {:6.2f}m, y: {:6.2f}m, z: {:6.2f}m\n\t \
            \Rotation Vector: rx: {:6.2f}deg, ry: {:6.2f}deg, rz: {:6.2f}deg".format(ids[i], x, y, z, rx, ry, rz))

    # if cv2.
            

# keepRecording = False
# recorder.join()
video.release()
tello.streamoff()

cv2.destroyAllWindows()