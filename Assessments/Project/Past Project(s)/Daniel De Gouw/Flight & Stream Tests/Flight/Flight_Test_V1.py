# Used to test how the drone reacts with different flight inputs
#
# Author: Daniel de Gouw
# Date: 9/4/19
import cv2
import cv2.aruco as aruco
import numpy as np
from pyparrot.Minidrone import Mambo

if __name__ == "__main__":
    mamboAddr = "e0:14:d0:63:3d:d0"
    
    # Retrieve one of the fiducial markers
    #dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)  
    
    mambo = Mambo(mamboAddr, use_wifi=True)

    print("trying to connect")
    success = mambo.connect(num_retries=3)
    print("connected: %s" % success)

    if (success):
        # get the state information
        print("sleeping")
        mambo.smart_sleep(2)
        mambo.ask_for_state_update()
        mambo.smart_sleep(2)
        
        # This is where we can do stuff now:
        
        print("taking off!")
        mambo.safe_takeoff(5)
        mambo.smart_sleep(1)
        
        counter = 0
        frame = 0;
        cv2.imshow('yo', frame)
        while counter < 10:
            mambo.fly_direct(roll=-25, pitch=-10, yaw=70, vertical_movement=0, duration=0.5) # Clockwise
            #mambo.fly_direct(roll=25, pitch=0, yaw=-70, vertical_movement=0, duration=2) # Anti-Clockwise
            mambo.smart_sleep(1)
            counter = counter + 1
            
            if cv2.waitKey(1) & 0xFF == ord('q'):         
                break            
        
        
        print("Landing!")
        mambo.safe_land(5)
        
    # Disconnect the drone
    print("disconnecting")
    mambo.disconnect()
    
    cv2.destroyAllWindows()