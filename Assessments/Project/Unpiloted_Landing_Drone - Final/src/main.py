#!python3.8

from djitellopy import Tello
from threading import Thread
from simple_pid import PID
import matplotlib.pyplot as plt
import cv2
import cv2.aruco as aruco
import pygame
import numpy as np
import time, sys
import cameraLoad
import FileIO


"""
    helpful links:

    https://github.com/damiafuentes/DJITelloPy.git
"""


# Speed of the drone - from external input
S = 60

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


# Angle Constants
YAW_THETA_SPEED   = 18  # [deg]
YAW_THETA_LIM   = 10     # [deg]
POS_THETA_LIM   = 5     # [deg]




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
            if (ids[i] == 23): #31=centre
                frame = aruco.drawAxis(frame, CAMERA_MATRIX, DISTORTION_COEFF, rvec[i], tvec[i], 0.035)
                return (ids[i], (round(x, 2), round(y, 2), round(z, 2)), round(rz, 2))

        

def avgFMCoord(storeIndex, storeVal):
    avgRY = sum(storeVal) / 10
    storeVal.clear()

    return avgRY
    
     

def yawControl(pidYaw, avgRZ, controlYaw):
    """
    Rotates the drone to degree 0.
    - Speed is given in units (cm/s)

    :param pidYaw: PID Func.
    :param avgRZ: Current rotation value. (-180 - +180) deg
    :param controlYaw: Current control yaw value.

    :param return: Control value.
    """
    _land_count = 0

    # Control the yaw angle
    # Compute new output from the PID according to the systems current value
    if (-YAW_THETA_LIM < int(avgRZ) and int(avgRZ) < YAW_THETA_LIM):   # do nothing 
        controlYaw = 0
        _land_count += 1
    elif (int(avgRZ) < -YAW_THETA_LIM or int(avgRZ) > YAW_THETA_LIM):  # rotate counter clkwise or clkwise 
        _delta_t = round((int(avgRZ) / YAW_THETA_SPEED), 2)
        controlYaw = int(pidYaw(_delta_t))
        if (controlYaw > 0):
            controlYaw += 10
        elif (controlYaw < 0):
            controlYaw -= 10

    return (controlYaw, _land_count)


def posControl(rc_control, tello, pidPos, coords, land_count, send_rc_land):
    """
    * Centring the drone to view - degree 0.

    # Control the positioning angle
    # Compute new output from the PID according 
        to the systems current value
    # if Marker not centred - update control movement

    # note:
    -  Have to invert Direction, due to image 
        being mirrored


    :param pidPos: PID Func.
    :param coords: Current x, y, z values.
    :param avgCP: Current centring angle.
    :param controlPos: Current control pos value.

    :param return: Control values.
    """
    x = int(coords[0]*100)  # m -> cm
    y = int(coords[1]*100)  # m -> cm
    z = 0
    _centred = 0

    y_view = -1
    x_view = -1

    _land_count = land_count
    
    #   note:
    #     - remember sign change   
    # left & right control - Horizontal
    if ((x < POS_THETA_LIM) and (x > -POS_THETA_LIM)):   # View centred - do nothing
        x = 0
        _land_count += 1
    elif ((x > POS_THETA_LIM) or (x < -POS_THETA_LIM)):  # View not centred
        x = -int(pidPos(x))
        if (x > 0):
            x += 10
        elif (x < 0):
            x -= 10

    # forward & back control - Vertical
    if ((y < POS_THETA_LIM) and (y > -POS_THETA_LIM)):   # View centred - do nothing
        y = 0
        _land_count += 1
    elif ((y > POS_THETA_LIM) or (y < -POS_THETA_LIM)):  # View not centred
        y = -int(pidPos(y))
        if (y > 0):
            y += 10
        elif (y < 0):
            y -= 10

    # Attempt to land
    if (_land_count == 3): # Land
        if (z < 15 and z > 10):
            tello.land()
            send_rc_land = True
            return (0, 0, 0)
        z = -((coords[2]*100)/3)-10


    return (int(x), int(y), int(z))



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
        self.send_rc_land = False

        # init PID - yaw  
        # self.pidYaw = PID(2, 0.2, 1.2, setpoint=0)  # Desired alpha angle = 0deg
        self.pidYaw = PID(1.5, 0.01, 0.3, setpoint=0)  # Desired alpha angle = 0deg
        self.pidYaw.sample_time = 0.1  # Update every 0.1 seconds
        self.controlYaw = 0

        # init PID - pos  
        # self.pidPos = PID(2, 0.1, 0.5, setpoint=0)  # Desired centre point angle = 0deg
        self.pidPos = PID(0.6, 0.01, 0.06, setpoint=0)
        # self.pidPos = PID(1, 0, 0, setpoint=0)
        self.pidPos.sample_time = 0.1  # Update every 0.1 seconds

        self.storeIndex = 0
        self.alpha = 0
        self.coords = (0, 0, 0)  # (x, y, z)
        self.storeRZ = []
        self.storeCP = []
        self.avgRZ = 0
        self.avgCP = 0
        # initilise the storing buffers for Plotting
        self.t0 = time.time()
        self.t_stamp = 0
        self.x_buff = np.array([])
        self.y_buff = np.array([])
        self.z_buff = np.array([])
        self.yaw_speed = np.array([])
        self.y_pos_data = np.array([])
        self.y_yaw_data = np.array([])

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
                    if (self.send_rc_control and not self.send_rc_land):
                        self.update()
                    else:
                        pass
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

            # Refresh frame
            self.screen.fill([0, 0, 0])  # Wipe Screen.
            frame = frame_read.frame     # Read a frame.


            self.send_rc_control = True

            txtBat = "Battery: ??%"
            txtX = "x: ??m"
            txtY = "y: ??m"
            txtZ = "z: ??m"
            txtRZ = "yaw angle: ??deg"
            txtCP = "Centre point angle: ??deg"

            try:
                txtBat = "Battery: {}%".format(self.tello.get_battery().strip())

                (ids, self.coords, rz) = detectFiducialMarker(frame)
        
                # Find the angle of the Markers centre point to the centre of the screen 
                self.alpha = round(np.degrees((np.arctan((self.coords[0]/self.coords[2])))), 2)  
                
                self.storeRZ.append(rz)
                self.storeCP.append(self.alpha)

                if (self.storeIndex >= 10):
                    self.avgRZ = avgFMCoord(self.storeIndex, self.storeRZ)
                    self.avgCP = avgFMCoord(self.storeIndex, self.storeCP)
                    self.storeIndex = 0
                else:
                    self.storeIndex += 1

                txtX = "x: {:6.2f}m".format(self.coords[0])
                txtY = "y: {:6.2f}m".format(self.coords[1])
                txtZ = "z: {:6.2f}m".format(self.coords[2])
                txtRZ = "yaw angle: {:6.2f}deg".format(self.avgRZ)           # Yaw angle
                txtCP = "Centre point angle: {:6.2f}deg".format(self.avgCP)

                self.send_rc_control = False  # if no markers are found - control will stay True.
                time.sleep(self.pidPos.sample_time)
            except AttributeError:
                pass
            except TypeError:
                pass

            # Draw text on screen
            cv2.putText(frame, txtBat, (5, 720 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.putText(frame, txtZ, (5, 720 - 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, txtY, (5, 720 - 105), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, txtX, (5, 720 - 140), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.putText(frame, txtRZ, (205, 720 - 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)    # Display yaw angle
            cv2.putText(frame, txtCP, (205, 720 - 105), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)   # Display centre point angle
                

            # Write frame to video file
            video.write(frame)  


            # Control based on ID position
            (self.controlYaw, land_count) = yawControl(self.pidYaw, self.avgRZ, self.controlYaw)    # yaw control  
            _coords = posControl(self.send_rc_control, self.tello, self.pidPos, self.coords, land_count, self.send_rc_land)    # pos control    
            self.tello.send_rc_control(_coords[0], _coords[1], _coords[2], self.controlYaw)  
            
            # Storing 'centring' & 'yaw' data in files
            tStep = round((time.time()-self.t0), 2)

            self.t_stamp = np.append(self.t_stamp, tStep)
            self.x_buff = np.append(self.x_buff, _coords[0])
            self.y_buff = np.append(self.y_buff, _coords[1])
            self.z_buff = np.append(self.z_buff, _coords[2])
            self.yaw_speed = np.append(self.yaw_speed, self.controlYaw)
            self.y_pos_data = np.append(self.y_pos_data, round(self.avgCP, 2))
            self.y_yaw_data = np.append(self.y_yaw_data, round(self.avgRZ, 2))

            # Writing to files
            FileIO.write_X_Y_Z_YAW(self.t_stamp, self.x_buff, self.y_buff, self.z_buff, self.yaw_speed, "src/data/xyz_plotting_data")
            FileIO.writeAngle(self.t_stamp, self.y_yaw_data, self.y_pos_data, "src/data/angle_plotting_data")


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
            time.sleep(2)
        elif key == pygame.K_l:  # land
            self.tello.land()
            self.send_rc_land = True


    def update(self):
        """ Update routine. Send velocities to Tello."""
        self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity, self.yaw_velocity)


def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()

