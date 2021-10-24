from djitellopy import tello
import numpy as np
import cv2
import cv2.aruco as aruco

tello = tello.Tello()
tello.connect()

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

camera_matrix = np.array([[  1.10203699e+03,   0.00000000e+00,   2.97856040e+02],
                          [  0.00000000e+00,   1.10715227e+03,   2.19618658e+02],
                          [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
distortion_coeff = np.array([7.978574896538845329e-02, 3.400042995004967317e+00, -1.786514214937548820e-02,
                            -3.217060871280347668e-03, -2.063856972981825777e+01])



print(tello.get_battery())
tello.streamon()


while True:
    frame = tello.get_frame_read().frame
    frame = cv2.resize(frame, (960, 720))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray, dictionary)  # Attempt to detect markers in the frame.
    
    if ids is not None:  # We have detected some markers.
        # Estimate the pose of the detected markers (assumes marker is 7cm wide).
        poses = aruco.estimatePoseSingleMarkers(corners, 0.035, camera_matrix, distortion_coeff)                          # Marker Area defined
        rvec, tvec = poses[0:2]
        frame = aruco.drawDetectedMarkers(frame, corners, ids)  # Draw the detected markers onto the frame.
        # Draw the estimated position for the detected markers and print the details in the terminal.
        for i in range(len(ids)):
            x, y, z = np.squeeze(tvec[i])
            rx, ry, rz = rotation_vector_to_euler(np.squeeze(rvec[i]))
            # Draw a 3D axis for each marker position.
            frame = aruco.drawAxis(frame, camera_matrix, distortion_coeff, rvec[i], tvec[i], 0.035)
            print("ID: {}\n\tTranslation Vector: x: {:6.2f}m, y: {:6.2f}m, z: {:6.2f}m\n\t \
            \Rotation Vector: rx: {:6.2f}deg, ry: {:6.2f}deg, rz: {:6.2f}deg".format(ids[i], x, y, z, rx, ry, rz))

    cv2.imshow('Fiducial', frame)



    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break




tello.end()
cv2.destroyAllWindows()

