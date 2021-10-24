# fiducial_detection_V3.py
#  This takes V2 and adds a controller skeleton of what to do with a debug output
#  Initialisation is performed which makes the drone do a 360 untill a Marker is found
#
# Author: Daniel de Gouw
# Date: 9/4/19
import numpy as np
import cv2
import cv2.aruco as aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)  # Retrieve one of the fiducial dictionaries.

def rotation_vector_to_euler(rotation_vector):
    #Convert an OpenCV-style rotation vector into Euler angles in degrees.
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    euler_angles, _, _, _, _, _ = cv2.RQDecomp3x3(rotation_matrix)
    return euler_angles

cap = cv2.VideoCapture(-1)  # Open the first camera connected to the computer.

# Normally these would be loaded from an external source.
camera_matrix = np.array([[  1.10203699e+03,   0.00000000e+00,   2.97856040e+02],
                          [  0.00000000e+00,   1.10715227e+03,   2.19618658e+02],
                          [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
distortion_coeff = np.array([7.978574896538845329e-02, 3.400042995004967317e+00, -1.786514214937548820e-02,
                            -3.217060871280347668e-03, -2.063856972981825777e+01])

ret, frame = cap.read()
h, w = frame.shape[0:2]
initialisation = True

#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#

while True:
    # Read an image from the frame.
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#-----------------------------------------------------------------------------------------------------#

    corners, ids, rejected = aruco.detectMarkers(gray, dictionary)  # Attempt to detect markers in the frame.
    
    # Marker detected
    if ids is not None:  # We have detected some markers.
        # Estimate the pose of the detected markers (assumes marker is 16cm wide).
        # Note: half the width gives correct distances...
        poses = aruco.estimatePoseSingleMarkers(corners, 0.08, camera_matrix, distortion_coeff)
        rvec, tvec = poses[0:2]
        frame = aruco.drawDetectedMarkers(frame, corners, ids)  # Draw the detected markers onto the frame.
        # Draw the estimated position for the detected markers and print the details in the terminal.
        for i in range(len(ids)):
            x, y, z = np.squeeze(tvec[i])
            rx, ry, rz = rotation_vector_to_euler(np.squeeze(rvec[i]))
            # Draw a 3D axis for each marker position.
            frame = aruco.drawAxis(frame, camera_matrix, distortion_coeff, rvec[i], tvec[i], 0.035)
            
            #print("ID: {}\n\tTranslation Vector: x: {:6.3f}m, y: {:6.3f}m, z: {:6.3f}m\n\t".format(ids[i], x, y, z))
    
        angle = np.degrees((np.arctan((x/z))))
        #print("\n %f \n" % angle)
        
        detected = True
        cv2.putText(frame, "Marker Detected", (10,20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
    
    # No Marker detected
    else:
        # Ensure the variables are reset and display text
        angle = 0
        detected = False

#-----------------------------------------------------------------------------------------------------#
# Controller for the drone
# If variable speed is possible, add in two sets of angles to prevent unstable system
    if initialisation == True and detected == False:
        # Call for sensor states:
        #    Check flying_state is not taking off / landing
        # If hovering then perform 360
        #    Choose a direction ---> LEFT
        print("360!! Need to find this marker")
    elif initialisation == True and detected == True:
        # delay 5 ms IF angle is less than some negative limit
        # Stop the drone
        # delay 5 ms
        initialisation = False
    elif detected == False:
        # Stop drone
        # re initialise after some delay
        cv2.putText(frame, "No Marker Detected", (10,20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
        print("Marker NOT Detected")
    elif detected and angle < 2.0 and angle > 0:
        # Stop - Do nothing
        cv2.putText(frame, "Centered", (10,50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
        print("Stopping")
    elif detected and angle > 2.0:
        # Check direction of turning
        # Stop if turning left
        # delay 5 ms
        # turn right
        cv2.putText(frame, "Turn Right", (10,50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
        print("Turning Right")
    elif detected and angle < 0:
        # Check direction of turning
        # Stop if turning right
        # delay 5 ms
        # turn left            
        cv2.putText(frame, "Turn Left", (10,50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
        print("Turning Left")
            
#-----------------------------------------------------------------------------------------------------#    
   
    # Draw axis for center point
    frame = cv2.line(frame, (0,int(h/2)), (w,int(h/2)), (0,255,0), 1) 
    frame = cv2.line(frame, (int(w/2),0), (int(w/2),h), (0,255,0), 1)
    
    # Display the frame
    cv2.imshow('Fiducial', frame)

#-----------------------------------------------------------------------------------------------------#

    # Close the script when q is pressed.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#

# Release the camera device and close the GUI.
cap.release()
cv2.destroyAllWindows()
