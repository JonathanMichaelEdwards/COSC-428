"""
Demo of the Bebop vision using DroneVisionGUI that relies on libVLC.  It is a different
multi-threaded approach than DroneVision

Author: Amy McGovern
"""
from pyparrot.Minidrone import Mambo
from pyparrot.DroneVisionGUI import DroneVisionGUI
import cv2, os, sys


# set this to true if you want to fly for the demo
testFlying = False
drone_image = None

class UserVision:
    def __init__(self, vision):
        self.index = 0
        self.vision = vision
        
    def save_pictures(self, args):
        #global drone_image
        
        drone_image = self.vision.get_latest_valid_picture()
        
    

def demo_mambo_user_vision_function(mamboVision, args):
    """
    Demo the user code to run with the run button for a mambo

    :param args:
    :return:
    """
    mambo = args[0]
    global drone_image
    if (testFlying):
        print("taking off!")
        mambo.safe_takeoff(5)
        test(mamboVision)
        
        if (mambo.sensors.flying_state != "emergency"):
            print("Flying state is %s" % mambo.sensors.flying_state)
            print("Flying direct: going up")
            mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=15, duration=2)

            print("flip left")
            print("flying state is %s" % mambo.sensors.flying_state)
            success = mambo.flip(direction="left")
            print("mambo flip result %s" % success)
            mambo.smart_sleep(5)
        test(mamboVision)
        print("landing")
        print("flying state is %s" % mambo.sensors.flying_state)
        mambo.safe_land(5)
    else:
        #print("Sleeeping for 15 seconds - move the mambo around")
        #mambo.smart_sleep(15)
        while True:
            if drone_image is not None:
                cv2.imshow('image', drone_image)
                drone_image=None
            else:
                print("not working")            
            mambo.smart_sleep(0.5)
            
    # done doing vision demo
    print("Ending the sleep and vision")
    mamboVision.close_video()

    mambo.smart_sleep(5)

    print("disconnecting")
    mambo.disconnect()


if __name__ == "__main__":
    if not os.geteuid() == 0:
        sys.exit("\nOnly root can run this script\n")    
        
    # you will need to change this to the address of YOUR mambo
    mamboAddr = "e0:14:d0:63:3d:d0"

    # make my mambo object
    # remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
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

        print("Preparing to open vision")
        mamboVision = DroneVisionGUI(mambo, is_bebop=False, buffer_size=200,
                                     user_code_to_run=demo_mambo_user_vision_function, user_args=(mambo, drone_image, ))
        userVision = UserVision(mamboVision)
        mamboVision.set_user_callback_function(userVision.save_pictures, user_callback_args=drone_image)
        mamboVision.open_video()