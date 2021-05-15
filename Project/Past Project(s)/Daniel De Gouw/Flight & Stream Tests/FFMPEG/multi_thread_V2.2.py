# Merging the the algorithm test code with the multi-threading code
#
# Author: Daniel de Gouw
# Date: 9/4/19
import threading
import PID
import time
import cv2, os
import cv2.aruco as aruco
import numpy as np
from pyparrot.Minidrone import Mambo
from subprocess import Popen, PIPE

# Set to true if testing with the Mambo
mambo_connected = True

# Set this to true if you want to fly for the demo
testFlying = False

# Debug Options
debug_controller = False
debug_draw_frame = True
debug_frame_output = True

#Other
drone_image = None
count = 0
keep_running=True

initialisation = True

controller_state = 0
angle_tracker = 0

# Normally these would be loaded from an external source.
camera_matrix = np.array([[  1.10203699e+03,   0.00000000e+00,   2.97856040e+02],
                          [  0.00000000e+00,   1.10715227e+03,   2.19618658e+02],
                          [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
distortion_coeff = np.array([7.978574896538845329e-02, 3.400042995004967317e+00, -1.786514214937548820e-02,
                            -3.217060871280347668e-03, -2.063856972981825777e+01])


class image_thread(threading.Thread):
    # Class for running the second thread
    def __init__(self):

        threading.Thread.__init__(self)

    def run(self):
        # Continuosly runs, obtaining frames from the stream
        global drone_image,count,keep_running
        # 'ffmpeg -i  rtsp://192.168.99.1/media/stream2 -async 1 -vsync 1 -vcodec rawvideo -pix_fmt rgb24 -f rawvideo -'
        cmd = 'ffmpeg -i  rtsp://192.168.99.1/media/stream2 -framerate 30 -vcodec rawvideo -pix_fmt rgb24 -f rawvideo -'
        p1 = Popen(cmd, stdout=PIPE, shell=True)
        fd = p1.stdout
        sx,sy=640,360
        while keep_running:
            # Reads the fram at a certain size
            s = fd.read(sx * sy * 3)
            drone_image = np.fromstring(s,dtype='uint8').reshape((sy,sx,3))
            #count = count + 1

def convert_image():
    # Copying the frame so that they are same for both conversions
    frame = drone_image
    
    # Converting the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    return frame, gray

def detect_markers(frame, gray):
    corners, ids, rejected = aruco.detectMarkers(gray, dictionary)  # Attempt to detect markers in the frame.
    
    # Marker detected
    if ids is not None:  # We have detected some markers.
        # Estimate the pose of the detected markers (assumes marker is 16cm wide).
        # Note: half the width gives correct distances...
        poses = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, distortion_coeff)
        rvec, tvec = poses[0:2]
        frame = aruco.drawDetectedMarkers(frame, corners, ids)  # Draw the detected markers onto the frame.
        # Draw the estimated position for the detected markers and print the details in the terminal.
        for i in range(len(ids)):
            x, y, z = np.squeeze(tvec[i])
            rx, ry, rz = rotation_vector_to_euler(np.squeeze(rvec[i]))
            # Draw a 3D axis for each marker position.
            frame = aruco.drawAxis(frame, camera_matrix, distortion_coeff, rvec[i], tvec[i], 0.035)
            
            #print("ID: {}\n\tTranslation Vector: x: {:6.3f}m, y: {:6.3f}m, z: {:6.3f}m\n\t".format(ids[i], x, y, z))
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

def debug_frame(frame, state, distance):
    w = 640
    h = 360
    # Draw axis for center point
    frame = cv2.line(frame, (0,int(h/2)), (w,int(h/2)), (0,255,0), 1) 
    frame = cv2.line(frame, (int(w/2),0), (int(w/2),h), (0,255,0), 1)
    
    cv2.putText(frame, "{:.2f}".format(distance), (300,300), cv2.FONT_HERSHEY_TRIPLEX, 5, (0,255,0), 2)
    
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
        cv2.putText(frame, "Aprox. 1m away", (10,80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 1)
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

def debug_controller_state(controller_state):
    # Prints an output of what state the tracking algorithm is in
    print("State: %2f \n" % controller_state)
    
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


def controller_fsm(controller_state, detected, distance, t_angle, r_angle, angle_count):
    # yaw - Y axis
    # pitch - X axis
    # roll - Z axis
    
    # Tracking algorithm for ensuring the Fiducial markers are centred, within a
    # certain distance (1 metre) and at a certain orientation (0 degrees)
    
    # Spins around till a marker is found
    if controller_state >= 0 and controller_state < 5:
        if detected == True:
            # Do nothing
            controller_state = 5
            angle_count = 0
        elif angle_count == 360:
            # Land
            mambo.smart_sleep(1)
            controller_state = 4
        else:
            angle_count = angle_count + 10
            # Rotate the drone left
            mambo.fly_direct(roll=0, pitch=0, yaw=40, vertical_movement=0, duration=0.1)
            mambo.smart_sleep(0.5)
    
    # Centres the marker within the frame    
    elif controller_state >= 5 and controller_state < 10:
        if detected == False:
            # Do nothing
            mambo.smart_sleep(1)
            pid_centring.update(pid_centring.SetPoint)
            pid_centring.ITerm = 0.0
            controller_state = 2
        elif t_angle > lowAngleLim and t_angle < upperAngleLim:
            # Do nothing
            mambo.smart_sleep(1)
            pid_centring.update(pid_centring.SetPoint)
            pid_centring.ITerm = 0.0
            controller_state = 10
        elif t_angle > upperAngleLim:
            # Right
            if controller_state == 7:
                pid_centring.ITerm = 0.0
            pid_centring.update(-t_angle)
            mambo.fly_direct(roll=0, pitch=0, yaw=pid_centring.output, vertical_movement=0, duration=0.5)
            mambo.smart_sleep(0.3)
            controller_state = 6
        elif t_angle < lowAngleLim:
            # Left
            if controller_state == 6:
                pid_centring.ITerm = 0.0
            pid_centring.update(-t_angle)
            mambo.fly_direct(roll=0, pitch=0, yaw=pid_centring.output, vertical_movement=0, duration=0.5)
            mambo.smart_sleep(0.3)
            controller_state = 7

    # Postions the drone so that the marker is approximately 1 metre away    
    elif controller_state >= 10 and controller_state < 15:
        if detected == False:
            # Do nothing
            mambo.smart_sleep(1)
            pid_distance.update(pid_distance.SetPoint)
            pid_distance.ITerm = 0.0            
            controller_state = 2
        elif t_angle < (lowAngleLim-2) or t_angle > (upperAngleLim+2):
            # Do nothing
            mambo.smart_sleep(1)
            pid_distance.update(pid_distance.SetPoint)
            pid_distance.ITerm = 0.0  
            controller_state = 9
        elif distance > 0.9 and distance < 1.1:
            # Do nothing
            mambo.smart_sleep(1)
            pid_distance.update(pid_distance.SetPoint)
            pid_distance.ITerm = 0.0
            controller_state = 15            
        elif distance > 1.1:
            # Move forward
            if controller_state == 12:
                pid_distance.ITerm = 0.0
            pid_distance.update(distance)
            if pid_distance.output < 15:
                forward = 15
            else:
                forward = -pid_distance.output
            mambo.fly_direct(roll=0, pitch=forward, yaw=0, vertical_movement=0, duration=0.4)
            mambo.smart_sleep(0.5)
            controller_state = 11
        elif distance < 0.9:
            # Move backward
            if controller_state == 11:
                pid_distance.ITerm = 0.0
                mambo.smart_sleep(1.5)
            pid_distance.update(distance)
            mambo.fly_direct(roll=0, pitch=-pid_distance.output*2, yaw=0, vertical_movement=0, duration=0.3)
            mambo.smart_sleep(0.3)
            controller_state = 12
            
    # Orientates the drone so that is perpendicular to the marker    
    elif controller_state >= 15 and controller_state < 20:
        if detected == False:
            # Do nothing
            mambo.smart_sleep(1)
            controller_state = 2
        elif t_angle < (lowAngleLim-2) or t_angle > (upperAngleLim+2):
            # Do nothing
            mambo.smart_sleep(1)
            controller_state = 9
        elif distance < 0.6 or distance > 1.4:
            # Do nothing
            mambo.smart_sleep(1)
            controller_state = 14
        elif r_angle > -20 and r_angle < 20:
            # Do nothing
            mambo.smart_sleep(1)
            controller_state = 19
        elif r_angle < -20:
            # Spin right 25 -30
            if controller_state == 17:
                mambo.smart_sleep(1)
            mambo.fly_direct(roll=20, pitch=-10, yaw=-18, vertical_movement=0, duration=0.1) # Anti-Clockwise
            mambo.smart_sleep(0.4)
            controller_state = 16
        elif r_angle > 20:
            # Spin left
            if controller_state == 16:
                mambo.smart_sleep(1)
            mambo.fly_direct(roll=-20, pitch=-10, yaw=15, vertical_movement=0, duration=0.1) # Clockwise
            mambo.smart_sleep(0.4)
            controller_state = 17
            
    
    return controller_state, angle_count


    
#-----------------------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------------------#


if __name__ == "__main__":
    
    # Fiducial marker library used
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)  # Retrieve one of the fiducial dictionaries.
    
    mamboAddr = "e0:14:d0:63:3d:d0"
    
    # PID initialisation
    pid_centring = PID.PID(5,1,1)
    pid_centring.setSampleTime(0.1)
    pid_centring.SetPoint = 1.0   
    
    pid_distance = PID.PID(15,1,1)
    pid_distance.setSampleTime(0.1)
    pid_distance.SetPoint = 1.0
    
    # Anges for alignment
    lowAngleLim = -1.0
    upperAngleLim = 3.0
    
    # make my mambo object
    mambo = Mambo(mamboAddr, use_wifi=True)
    print("trying to connect to mambo now")
    success = mambo.connect(num_retries=3)
    print("connected: %s" % success)
    
    if (success):
        # get the state information
        print("sleeping")
        mambo.smart_sleep(1)
        mambo.ask_for_state_update()
        mambo.smart_sleep(1)
        
        # Start the vision thread
        Obtain_Image_Thread = image_thread()
        Obtain_Image_Thread.start()     
        
        mambo.smart_sleep(5)
        print("Taking off!!")
        #mambo.safe_takeoff(5)
        mambo.smart_sleep(1)

        
    while True:
    
        if drone_image is not None:
            # Get the BGR image and grayscale image
            frame, gray = convert_image()
            
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
                
            print("State: %2f \n" % controller_state)
            print("Distance: %.2f\n" % co_ords[2])
            print("t_angle: %.2f\n" % t_angle)
            print("r_angle: %.2f\n" % r_angle)
            print("PID centre: %.2f \n" % pid_centring.output)
            #print("With I: %.2f \n" % pid_centring.ITerm)
            print("PID distance: %.2f \n" % pid_distance.output)
            #print("With I: %.2f \n" % pid_distance.ITerm)            
            
            # Controller
            controller_state, angle_tracker = controller_fsm(controller_state, detected, co_ords[2], t_angle, r_angle, angle_tracker)
        
            # Debug the controller
            if debug_controller:
                debug_controller_state(controller_state)
                
            # Debug and draw on the frame
            if debug_draw_frame:
                frame = debug_frame(frame, controller_state, t_angle)
            
            # Display the frame for de bugging
            if debug_frame_output:
                cv2.imshow('Drones View', frame)
                
                # Resetting the variable - we will then wait for a new frame
                drone_image=None
        #else:
            #print("No new image!! Stream error")
        
        # Always check forq button press or end of code
        if ((cv2.waitKey(1) & 0xFF == ord('q')) or (controller_state == 4)):
            # Stop thread on next loop
            keep_running=False
            mambo.safe_land(5)
            mambo.smart_sleep(2)
            # Break this while loop
            break
    
# Wait for thread to finish and close it
Obtain_Image_Thread.join()

#Close all open windows
cv2.destroyAllWindows()

# Disconnect the drone
mambo.smart_sleep(1)
print("Disconnecting from Drone")
mambo.disconnect()



