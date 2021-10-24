#!python3.8

from djitellopy import Tello
from threading import Thread
from simple_pid import PID
import cv2
import cv2.aruco as aruco
import pygame
import numpy as np
import time, sys
import cameraLoad


# Speed of the drone
S = 20

# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
FPS = 120

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


def detectFiducialMarker(frame):
    """
        Algorithm for detecting a Fiducial marker.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, dictionary)  # Attempt to detect markers in the frame.
    if ids is not None:  # We have detected some markers.
        # Estimate the pose of the detected markers (assumes marker is 7cm wide).
        poses = aruco.estimatePoseSingleMarkers(corners, 0.035, CAMERA_MATRIX, DISTORTION_COEFF)  # Marker Area defined
        rvec, tvec = poses[0:2]

        frame = aruco.drawDetectedMarkers(frame, corners, ids)  # Draw the detected markers onto the frame.
        
        # Draw the estimated position for the detected markers and print the details in the terminal.
        for i in range(len(ids)):
            x, y, z = np.squeeze(tvec[i])
            rx, ry, rz = rotation_vector_to_euler(np.squeeze(rvec[i]))
            # Draw a 3D axis for each marker position.
            frame = aruco.drawAxis(frame, CAMERA_MATRIX, DISTORTION_COEFF, rvec[i], tvec[i], 0.035)
            # print("ID: {}\n\tTranslation Vector: x: {:6.2f}m, y: {:6.2f}m, z: {:6.2f}m\n\t \
            # \Rotation Vector: rx: {:6.2f}deg, ry: {:6.2f}deg, rz: {:6.2f}deg".format(ids[i], x, y, z, rx, ry, rz))


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



controller_state = 0
angle_tracker = 0

def fsmController():
    # (Movement stage)
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




class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations (yaw)
            - W and S: Up and down.
    """

    def __init__(self):
        # Init pygame
        pygame.init()

        # Create pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        self.send_rc_control = False

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)


    def run(self):

        self.tello.connect()

        # In case streaming is on. This happens when we quit this program without the escape key.
        self.tello.streamoff()
        self.tello.streamon()

        self.tello.set_speed(self.speed)

        frame_read = self.tello.get_frame_read()
        height, width, _ = frame_read.frame.shape
        video = cv2.VideoWriter('video/video.avi', cv2.VideoWriter_fourcc(*'MJPG'), 14, (width, height))

        should_stop = False
        while not should_stop:
            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                break



            ############################## -- Algorithm
            # self.tello.get_barometer() # Get Altitude

            # self.tello.send_rc_control(0, 0, 0, 90)
            # time.sleep(1)

            self.screen.fill([0, 0, 0])  # Wipe Screen.
            frame = frame_read.frame  # Read a frame.
            try:
                text = "Battery: {}%".format(self.tello.get_battery().strip())
            except AttributeError:
                text = "Battery: ??%"
            cv2.putText(frame, text, (5, 720 - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            detectFiducialMarker(frame)
            # video.write(frame)  # Write frame to video file


            
            # Controller
            controller_state, angle_tracker = fsmController(controller_state, detected, co_ords[2], t_angle, r_angle, angle_tracker)
            ##############################


        
            # Update frame.
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)
            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()
            time.sleep(1 / FPS)


        # Closing Windows
        pygame.display.quit()
        pygame.quit()
        cv2.destroyAllWindows()

        # Dealocating Resources
        keepRecording = False
        self.tello.end()
        video.release()
        # recorder.join()
        sys.exit(0)



    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S


    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.tello.send_rc_control(0, 0, 0, 0)
            time.sleep(5)
            self.send_rc_control = True
            self.tello.send_rc_control(0, 0, 0, 23)
            time.sleep(1.1) # θ rate per second + freq(IMU Read speed) ~10 Hz
            self.tello.send_rc_control(0, 0, 0, 0)
            time.sleep(5)
        elif key == pygame.K_l:  # land
            not self.tello.land()
            self.send_rc_control = False


    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                                       self.up_down_velocity, self.yaw_velocity)


def main():
    # init PID  
    pid_centring = PID.PID(10, 1, 1)
    pid_centring.setSampleTime(0.1)
    pid_centring.SetPoint = 1.0   

    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()

