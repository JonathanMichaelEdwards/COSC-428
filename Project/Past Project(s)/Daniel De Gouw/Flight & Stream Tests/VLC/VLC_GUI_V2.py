"""
Demo of the Bebop vision using DroneVisionGUI that relies on libVLC.  It is a different
multi-threaded approach than DroneVision
"""
from pyparrot.Minidrone import Mambo
from pyparrot.DroneVisionGUI import DroneVisionGUI
import cv2, os, sys
import cv2.aruco as aruco
import numpy as np

# Set to true if testing with the Mambo
mambo_connected = True

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

#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#           
    
def capture_image():
    # Read an image from the frame.
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    return ret, frame, gray

def read_image():
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
            
            print("ID: {}\n\tTranslation Vector: x: {:6.3f}m, y: {:6.3f}m, z: {:6.3f}m\n\t".format(ids[i], x, y, z))
            #print("ID: {}\n\tTranslation Vector: x: {:6.3f}m, y: {:6.3f}m, z: {:6.3f}m\n\t \
            #\Rotation Vector: rx: {:6.2f}deg, ry: {:6.2f}deg, rz: {:6.2f}deg".format(ids[i], x, y, z, rx, ry, rz))            
        
        detected = True
        co_ords = (x, y, z, ry)
    
    # No Marker detected
    else:
        # Ensure the variables are reset
        co_ords = (0, 0, 0, 0)
        detected = False
    
    return frame, co_ords, detected
    
def rotation_vector_to_euler(rotation_vector):
    # Convert an OpenCV-style rotation vector into Euler angles in degrees.
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    euler_angles, _, _, _, _, _ = cv2.RQDecomp3x3(rotation_matrix)
    return euler_angles

def debug_frame(frame, state):
    # Draw axis for center point
    frame = cv2.line(frame, (0,int(h/2)), (w,int(h/2)), (0,255,0), 1) 
    frame = cv2.line(frame, (int(w/2),0), (int(w/2),h), (0,255,0), 1)
    
    if state >= 0 and state < 5:
        cv2.putText(frame, "No Marker Detected", (10,20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
    else:
        cv2.putText(frame, "Marker Detected", (10,20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
        
    if state >= 10 and state < 15:
        cv2.putText(frame, "Centered", (10,50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
    elif state == 6:
        cv2.putText(frame, "Turn Right", (10,50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
    elif state == 7:
        cv2.putText(frame, "Turn Left", (10,50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)            
    
    if state >= 15 and state < 20:
        cv2.putText(frame, "Aprox. 1m away", (10,50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
    elif state == 11:
        cv2.putText(frame, "Moving Forward", (10,80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
    elif state == 12:
        cv2.putText(frame, "Moving Backwards", (10,80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)    
    
    if state == 16:
        cv2.putText(frame, "Spinning Right", (10,110), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
    elif state == 17:
        cv2.putText(frame, "Spinning Left", (10,110), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
    elif state == 19:
        cv2.putText(frame, "Square on", (10,110), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)        
    
    
    return frame

def controller_fsm(controller_state, detected, distance, t_angle, r_angle, angle_count):
    # yaw - Y axis
    # pitch - X axis
    # roll - Z axis
    
    if controller_state >= 0 and controller_state < 5:
        if detected == True:
            # Do nothing
            controller_state = 5
            angle_count = 0
        elif angle_count == 360:
            # Land
            controller_state = 4
            
        #angle_count = angle_count + 20
        # Rotate the drone left
        
    elif controller_state >= 5 and controller_state < 10:
        if detected == False:
            # Do nothing
            controller_state = 2
        elif t_angle > 0 and t_angle < 2.0:
            # Do nothing
            controller_state = 10
        elif t_angle > 2.0:
            # Right
            controller_state = 6
        elif t_angle < 0:
            # Left
            controller_state = 7

    elif controller_state >= 10 and controller_state < 15:
        if detected == False:
            # Do nothing
            controller_state = 2
        elif t_angle < 0 or t_angle > 2.0:
            # Do nothing
            controller_state = 9
        elif distance > 1:
            # Move forward
            controller_state = 11
        elif distance < 0.8:
            # Move backward
            controller_state = 12
        else:
            # Do nothing
            controller_state = 15  
                
        # Use pitch and yaw to go in semi circle
    elif controller_state >= 15 and controller_state < 20:
        if detected == False:
            # Do nothing
            controller_state = 2
        elif t_angle < 0 or t_angle > 2.0:
            # Do nothing
            controller_state = 9
        elif r_angle < -10:
            # Spin right
            controller_state = 16
        elif r_angle > 10:
            # Spin left
            controller_state = 17
        else:
            # Do nothing
            controller_state = 19
    
    # Set the speed of the controller here
    #mambo.smart_sleep(1)
    
    return controller_state, angle_count


def debug_controller_state(controller_state):
    print("State: %2.0f \n" % controller_state)
    
    if controller_state >= 0 and controller_state < 5:
        print("Detecting state \n")
        if controller_state == 1:
            print("Spinning around to the left 10 deg/loop")
        elif controller_state == 2:
            print("Marker lost - Attempting to find it again ")
        elif controller_state == 4:
            print("Landing - No marker found")
            
    elif controller_state >= 5 and controller_state < 10:
        print("Aligning state \n")
        if controller_state == 9:
            print("Alignment lost in another state - Realigning")
        elif controller_state == 6:
            print("Turning Right")
        elif controller_state == 7:
            print("Turning Left")
            
    elif controller_state >= 10 and controller_state < 15:
        print("Adjusting State \n")
        if controller_state == 11:
            print("Moving forward")
        elif controller_state == 12:
            print("Moving backward")
    
    print("\n") # spacer
    
#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#
def main_program(mamboVision, args):
    initialisation = True
    
    controller_state = 0
    angle_tracker = 0    
    while True:
        #mambo.smart_sleep(1)
        #retrieve_image(mamboVision)
        frame = mamboVision.get_latest_valid_picture()
        #cv2.imshow('name', img)
        if not frame.all() == None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Look for Fiducial Markers and return the co-ordinates
            frame, co_ords, detected = detect_markers(frame, gray)
            
            # Find the angle of the Markers centre point to the centre of the screen - t_angle
            # Find the angle of the markers reference frame wrt camera reference frame - r_angle
            if not co_ords == (0, 0, 0, 0):
                t_angle = np.degrees((np.arctan((co_ords[0]/co_ords[2]))))
                r_angle = co_ords[3]
                #print("\n %f \n" % r_angle)
            else:
                # No marker detected
                t_angle = 0; # To mitigate 0 division error
                r_angle = 0; # For completeness
            
            # Controller
            controller_state, angle_tracker = controller_fsm(controller_state, detected, co_ords[2], t_angle, r_angle, angle_tracker)
            
            # Debug the controller
            if debug_controller:
                debug_controller_state(controller_state)
                
            # Debug and draw on the frame
            if debug_draw_frame:
                frame = debug_frame(frame, controller_state)
            
            # Display the frame for de bugging
            if debug_frame_output:
                cv2.imshow('Fiducial', frame)
        
            # Close the script when q is pressed or the controller decides to stop.
            if ((cv2.waitKey(1) & 0xFF == ord('q')) or (controller_state == 4)):
                debug_controller = False
                if mambo_connected:
                    mambo.smart_sleep(2)            
                break

    # done doing vision demo
    print("Ending the sleep and vision")
    mamboVision.close_video()

    mambo.smart_sleep(5)

    print("disconnecting")
    mambo.disconnect()
    
    
    
    
    
if __name__ == "__main__":

    if not os.geteuid() == 0:
        sys.exit("\nOnly root can run this script\n")  
        
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
            print("Preparing to open vision")
            mamboVision = DroneVisionGUI(mambo, is_bebop=False, buffer_size=200,
                                         user_code_to_run=main_program, user_args=(mambo, ), fps=30)
            mamboVision.open_video()
"""            
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
        
        # Find the angle of the Markers centre point to the centre of the screen - t_angle
        # Find the angle of the markers reference frame wrt camera reference frame - r_angle
        if not co_ords == (0, 0, 0, 0):
            t_angle = np.degrees((np.arctan((co_ords[0]/co_ords[2]))))
            r_angle = co_ords[3]
            #print("\n %f \n" % r_angle)
        else:
            # No marker detected
            t_angle = 0; # To mitigate 0 division error
            r_angle = 0; # For completeness
        
        # Controller
        controller_state, angle_tracker = controller_fsm(controller_state, detected, co_ords[2], t_angle, r_angle, angle_tracker)
        
        # Debug the controller
        if debug_controller:
            debug_controller_state(controller_state)
            
        # Debug and draw on the frame
        if debug_draw_frame:
            frame = debug_frame(frame, controller_state)
        
        # Display the frame for de bugging
        if debug_frame_output:
            cv2.imshow('Fiducial', frame)
    
        # Close the script when q is pressed or the controller decides to stop.
        if ((cv2.waitKey(1) & 0xFF == ord('q')) or (controller_state == 4)):
            debug_controller = False
            if mambo_connected:
                mambo.smart_sleep(2)            
            break

#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#
    if mambo_connected:
        # Stop saving images from the drone
        print("Ending drone vision")
        mamboVision.close_video()
        
        mambo.smart_sleep(5)
        
        # Disconnect the drone
        print("disconnecting")
        mambo.disconnect()
        
    else:
        # Release the computer camera device and close the GUI.
        cap.release()
        
        if debug_frame_output:
            # Close the debug window
            cv2.destroyAllWindows()
"""