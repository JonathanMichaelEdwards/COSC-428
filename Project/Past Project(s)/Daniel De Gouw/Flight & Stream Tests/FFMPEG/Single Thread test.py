# Testing the FFmpeg on a single thread
#
# Author: Daniel de Gouw
# Date: 9/4/19
import cv2,os
import numpy as np
from subprocess import Popen, PIPE
import cv2.aruco as aruco
import numpy as np
from pyparrot.Minidrone import Mambo

mamboAddr = "e0:14:d0:63:3d:d0"

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)  # Retrieve one of the fiducial

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


def use_fiduical(input_img):
    frame = input_img
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    see_fiduical = False
    z = False
    corners, ids, rejected = aruco.detectMarkers(gray, dictionary)  # Attempt to detect markers in the frame.

    if ids is not None:  # We have detected some markers.
    # Estimate the pose of the detected markers (assumes marker is 7cm wide).
	    poses = aruco.estimatePoseSingleMarkers(corners, 0.035, camera_matrix, distortion_coeff)
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
		    see_fiduical = True
    else:
	    print('No marker detected')
    #cv2.imshow('Fiducial', frame)

    return frame,see_fiduical,z

    

if __name__ == "__main__":

    #cmd = 'ffmpeg -f v4l2 -i /dev/video0 -vcodec rawvideo -pix_fmt rgb24 -f rawvideo -'
    #cmd ='ffplay rtsp://192.168.99.1/media/stream2'
    #cmd = 'ffmpeg -i  rtsp://192.168.99.1/media/stream2 -vcodec rawvideo -pix_fmt rgb24 -f rawvideo -'
    cmd = 'ffmpeg -i  rtsp://192.168.99.1/media/stream2 -vcodec rawvideo -pix_fmt rgb24 -f rawvideo -'

    #fd = os.popen(cmd)
    p1 = Popen(cmd, stdout=PIPE, shell=True)
    fd = p1.stdout

    sx,sy=640,360

    mambo = Mambo(mamboAddr, use_wifi=True)

    print("trying to connect")
    success = mambo.connect(num_retries=3)
    print("connected: %s" % success)

    if (success):
        # get the state information
        print("sleeping")
        mambo.smart_sleep(2)
        mambo.ask_for_state_update()
        mambo.smart_sleep(2)

        print("taking off!")
        mambo.safe_takeoff(5)
    
    while True:
        #print('--1-')
        s=fd.read(sx*sy*3)
        #print('--2-')
        img = np.fromstring(s,dtype='uint8').reshape((sy,sx,3))
        cv2.imshow('image',img)
        img,see_or_not,distance = use_fiduical(img)
        c=cv2.waitKey(1)
        if c%256==ord('q'):
            mambo.safe_land(5)
            mambo.smart_sleep(2)			
            break
        if see_or_not == True and distance <= 1:
            mambo.safe_land(5)
            mambo.smart_sleep(2)
            break
        if see_or_not == True and distance > 1:
            mambo.fly_direct(roll=0, pitch=10, yaw=0, vertical_movement=0, duration=0.1)
    
    cv2.destroyAllWindows()
    
    
    
