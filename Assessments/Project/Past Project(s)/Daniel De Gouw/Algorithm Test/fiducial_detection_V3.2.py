# fiducial_detection_V3.py
#  This takes V3 and creates a debug function for the HUD, a marker detection function
#  and a capture frame function
#
# Author: Daniel de Gouw
# Date: 9/4/19
import numpy as np
import cv2
import cv2.aruco as aruco
from pyparrot.Minidrone import Mambo

# Mambo MAC address
mamboAddr = "e0:14:d0:63:3d:d0"

# Fiducial marker library used
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)  # Retrieve one of the fiducial dictionaries.

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

controller_state = 0
tracker = 0
#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#

def capture_image():
    # Read an image from the frame.
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return ret, frame, gray

def detect_markers(frame, gray):
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
        
        detected = True
        co_ords = (x, y, z)
    
    # No Marker detected
    else:
        # Ensure the variables are reset
        co_ords = (0, 0, 0)
        detected = False
    
    return frame, co_ords, detected
    
def rotation_vector_to_euler(rotation_vector):
    #Convert an OpenCV-style rotation vector into Euler angles in degrees.
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    euler_angles, _, _, _, _, _ = cv2.RQDecomp3x3(rotation_matrix)
    return euler_angles

def debug_frame(frame, detected, angle):
    # Draw axis for center point
    frame = cv2.line(frame, (0,int(h/2)), (w,int(h/2)), (0,255,0), 1) 
    frame = cv2.line(frame, (int(w/2),0), (int(w/2),h), (0,255,0), 1)
    
    if detected == False:
        cv2.putText(frame, "No Marker Detected", (10,20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
    elif detected == True:
        cv2.putText(frame, "Marker Detected", (10,20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
        
        if angle < 2.0 and angle > 0:
            cv2.putText(frame, "Centered", (10,50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
        elif angle > 2.0:
            cv2.putText(frame, "Turn Right", (10,50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
        elif angle < 0:
            cv2.putText(frame, "Turn Left", (10,50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)        
    
    #if not angle == None:
    
    
    return frame

def controller_fsm(controller_state, detected, distance, angle, angle_count):
    # The tracking algorithm used to keep the marker in the screen at a certain distance
    print("\n State: %.0f \n" % controller_state)
    
    if controller_state == 0:
        print("turning 10 degrees left untill marker is found")
        angle_count = angle_count + 10
        if detected == True:
            print("Marker found, stopping in 1 second")
            controller_state = 1;
            angle_count = 0;
        elif angle_count == 360:
            print("Landing, no marker found")
    elif controller_state == 1:
        print("Aligning with marker")
        if detected == False:
            print("Marker Lost")
            controller_state = 0
        elif angle < 2.0 and angle > 0:
            print("Centered")
            controller_state = 2
        elif angle > 2.0:
            print("Turning Right")
        elif angle < 0:           
            print("Turning Left")
    elif controller_state == 2:
        print("adjusting distance")
        if detected == False:
            print("Marker Lost")
            controller_state = 0
        elif angle > 2.0 or angle < 0:
            print("Need to realign")
            controller_state = 1
        elif distance > 1:
            print("Moving forward")
        elif distance < 0.8:
            print("Moving backward")
        else:
            print("Adjusted successfully")
            
    return controller_state, angle_count

def controller(initialisation, detected, angle, distance):
    # Controller for the drone
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
            print("Marker NOT Detected")
        elif detected:
            if angle > 2.0:
                # Check direction of turning
                # Stop if turning left
                # delay 5 ms
                # turn right
                print("Turning Right")
            elif angle < 0:
                # Check direction of turning
                # Stop if turning right
                # delay 5 ms
                # turn left            
                print("Turning Left")
            elif distance > 1:
                print("Moving forward")
            elif distance < 0.8:
                print("Moving backward")
            else:
                print("Adjusted successfully")    


#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#

while True:
    # Get the latest frame ing BGR and gray-scale
    ret, frame, gray = capture_image()

#-----------------------------------------------------------------------------------------------------#

    # Detecting for Fiducials and getting their Co-ords
    frame, co_ords, detected = detect_markers(frame, gray)
    
    if not co_ords == (0, 0, 0):
        angle = np.degrees((np.arctan((co_ords[0]/co_ords[2]))))
    else:
        angle = 0;
    
    #print("\n %f \n" % angle)
    
    
#-----------------------------------------------------------------------------------------------------#
# Controller
    # FSM
    controller_state, tracker = controller_fsm(controller_state, detected, co_ords[2], angle, tracker)
    
    # Process
    #controller(initialisation, detected, angle, co_ords[2])
    
#-----------------------------------------------------------------------------------------------------#    
    
    # Disable this for raw view of frame
    frame = debug_frame(frame, detected, angle)
    
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