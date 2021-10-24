"""
Demo of the ffmpeg based mambo vision code (basically flies around and saves out photos as it flies)

Author: Amy McGovern
"""
import cv2
from pyparrot.Minidrone import Mambo
from pyparrot.DroneVision import DroneVision
import threading
import time

# set this to true if you want to fly for the demo
testFlying = False

# You will need to change this to the address of YOUR mambo
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
    mamboVision = DroneVision(mambo, is_bebop=False, buffer_size=30)
    success = mamboVision.open_video()
    print("Success in opening vision is %s" % success)

    if (success):
        while True:
            mambo.smart_sleep(1)
            frame = mamboVision.get_latest_valid_picture()
            
            cv2.imshow('Test', frame)

        mamboVision.close_video()
        mambo.smart_sleep(5)
        
        cv2.destroyAllWindows()

    print("disconnecting")
    mambo.disconnect()
