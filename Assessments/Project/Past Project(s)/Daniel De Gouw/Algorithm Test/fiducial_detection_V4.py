# fiducial_detection_V3.py
#  This takes V3 and creates a debug function for the HUD, a marker detection function
#  and a capture frame function
#
# Author: Daniel de Gouw
# Date: 9/4/19
import cv2
import threading
import time
import cv2.aruco as aruco
import numpy as np
from pyparrot.Minidrone import Mambo
from pyparrot.DroneVision import DroneVision

# Set to true if testing with the Mambo
mambo_connected = False

# Set this to true if you want to fly for the demo
testFlying = False

# Debug Options
debug_controller = True
debug_draw_frame = True
debug_frame_output = True

# Mambo MAC address
mamboAddr = "e0:14:d0:63:3d:d0"

# Fiducial marker library used
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)  # Retrieve one of the fiducial dictionaries.
    
# Normally these would be loaded from an external source.
camera_matrix = np.array([[  1.10203699e+03,   0.00000000e+00,   2.97856040e+02],
                          [  0.00000000e+00,   1.10715227e+03,   2.19618658e+02],
                          [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
distortion_coeff = np.array([7.978574896538845329e-02, 3.400042995004967317e+00, -1.786514214937548820e-02,
                            -3.217060871280347668e-03, -2.063856972981825777e+01])

# Initialising Variables
controller_state = 0
angle_tracker = 0

#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#

class UserVision:
    # Class used when the drone is connected to the computer as the source for 
    # the images. Tries to save each frame so that the other code can read it    
    def __init__(self, vision):
        self.vision = vision

    def save_pictures(self, args):
        img = self.vision.get_latest_valid_picture()
        
        if (img is not None):
            filename = "drone_img.png"
            cv2.imwrite(filename, img)
            
    
def capture_image():
    # Read an image from the frame.
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    return ret, frame, gray

def read_image():
    # Trying to read from the image saved by save_pictures
    frame = cv2.imread("drone_img.png", CV_LOAD_IMAGE_COLOR)
    
    if not frame is None:
        ret = True
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        ret = False
        
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
    # Convert an OpenCV-style rotation vector into Euler angles in degrees.
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
    
    return frame

def controller_fsm(controller_state, detected, distance, angle, angle_count):
    # The tracking algorithm used to keep the marker in the screen at a certain distance
    # and at a certain orientation     

    if debug_controller:
        print("\n State: %.0f \n" % controller_state)
    
    if controller_state == 0:
        if debug_controller:
            print("turning 10 degrees left/loop untill marker is found")
        angle_count = angle_count + 10
        if detected == True:
            if debug_controller:
                    print("Marker found, stopping in 1 second")
            controller_state = 1;
            angle_count = 0;
        elif angle_count == 360:
            if debug_controller:
                    print("Landing, no marker found")
    elif controller_state == 1:
        if debug_controller:
            print("Aligning with marker")
        if detected == False:
            if debug_controller:
                    print("Marker Lost")
            controller_state = 0
        elif angle < 2.0 and angle > 0:
            if debug_controller:
                    print("Centered")
            controller_state = 2
        elif angle > 2.0:
            if debug_controller:
                    print("Turning Right")
        elif angle < 0:           
            if debug_controller:
                    print("Turning Left")
    elif controller_state == 2:
        if debug_controller:
            print("adjusting distance")
        if detected == False:
            if debug_controller:
                    print("Marker Lost")
            controller_state = 0
        elif angle > 2.0 or angle < 0:
            if debug_controller:
                    print("Need to realign")
            controller_state = 1
        elif distance > 1:
            if debug_controller:
                    print("Moving forward")
        elif distance < 0.8:
            if debug_controller:
                    print("Moving backward")
        else:
            if debug_controller:
                    print("Adjusted successfully")
            
    return controller_state, angle_count

#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#

if __name__ == "__main__":
    
    # Initialising the code depending on the drone being connected
    if mambo_connected:
        mambo = Mambo(mamboAddr, use_wifi=True)
        print("Trying to connect")
        success = mambo.connect(num_retries=3)
        print("Connected: %s" % success)
        
        if (success):    
            # Get the state information
            print("Sleeping")
            mambo.smart_sleep(1)
            mambo.ask_for_state_update()
            mambo.smart_sleep(1)
            # Set the drone to send images over
            print("Preparing to open vision")
            mamboVision = DroneVision(mambo, is_bebop=False, buffer_size=30)
            userVision = UserVision(mamboVision)
            mamboVision.set_user_callback_function(userVision.save_pictures, user_callback_args=None)
            success = mamboVision.open_video()
            print("Success in opening vision is %s" % success)
            
            # Read the first image
            if (success):
                print("Vision successfully started!") 
                ret, frame, gray = read_image()        
    else:
        print("Set to computer camera")
        # Reading an image from the camera on computer
        cap = cv2.VideoCapture(-1)
        ret, frame, gray = capture_image()
    
    # Find the size of the image in pixels
    h, w = frame.shape[0:2]                    
    
    # Main loop
    while True:
        # Capture an Image
        if mambo_connected:
            ret, frame, gray = read_image()
        else:
            ret, frame, gray = capture_image()
        
        # Look for Fiducial Markers and return the co-ordinates
        frame, co_ords, detected = detect_markers(frame, gray)
        
        # Find the angle of the Markers centre point to the centre of the screen
        if not co_ords == (0, 0, 0):
            angle = np.degrees((np.arctan((co_ords[0]/co_ords[2]))))
            #print("\n %f \n" % angle)
        else:
            # No marker detected
            angle = 0;        
        
        # Controller
        controller_state, angle_tracker = controller_fsm(controller_state, detected, co_ords[2], angle, angle_tracker)
        
        # Disable this for raw view of frame
        if debug_draw_frame:
            frame = debug_frame(frame, detected, angle)
        
        # Display the frame
        if debug_frame_output:
            cv2.imshow('Fiducial', frame)
    
        # Close the script when q is pressed.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#
    
    # Release the camera device and close the GUI.
    cap.release()
    cv2.destroyAllWindows()