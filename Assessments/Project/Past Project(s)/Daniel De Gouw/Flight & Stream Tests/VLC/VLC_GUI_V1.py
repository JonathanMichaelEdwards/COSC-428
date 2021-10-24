"""
Demo of the Bebop vision using DroneVisionGUI that relies on libVLC.  It is a different
multi-threaded approach than DroneVision
"""
from pyparrot.Minidrone import Mambo
from pyparrot.DroneVisionGUI import DroneVisionGUI
import cv2, os, sys


# set this to true if you want to fly for the demo
testFlying = False


def retrieve_image(vision):
    #img = vision.get_latest_valid_picture()
    #print(img)
    #cv2.imshow('name', img)
    return
    

def main_program(mamboVision, args):
    
    while True:
        mambo.smart_sleep(1)
        #retrieve_image(mamboVision)
        img = mamboVision.get_latest_valid_picture()
        cv2.imshow('name', img)        
        # Close the script when q is pressed
        #if cv2.waitKey(1) & 0xFF == ord('q'):
            #break     

    # done doing vision demo
    print("Ending the sleep and vision")
    mamboVision.close_video()

    mambo.smart_sleep(5)

    print("disconnecting")
    mambo.disconnect()


if __name__ == "__main__":

    if not os.geteuid() == 0:
        sys.exit("\nOnly root can run this script\n")      
    
    mamboAddr = "e0:14:d0:63:3d:d0"

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

        print("Preparing to open vision")
        mamboVision = DroneVisionGUI(mambo, is_bebop=False, buffer_size=200, network_caching=200,
                                     user_code_to_run=main_program, user_args=(mambo, ), fps=10)
        mamboVision.open_video()
        