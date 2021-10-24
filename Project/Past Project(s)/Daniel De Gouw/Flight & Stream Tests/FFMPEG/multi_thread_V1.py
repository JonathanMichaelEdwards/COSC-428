# Testing out multithreading with Parrot Mambo
#
# Author: Daniel de Gouw
# Date: 9/4/19
import threading
import time
import cv2,os
import numpy as np
from subprocess import Popen, PIPE
import cv2.aruco as aruco
import numpy as np
from pyparrot.Minidrone import Mambo


drone_image = None
count = 0
keep_running=True

class image_thread(threading.Thread):
    # Class for running the second thread
    def __init__(self):

        threading.Thread.__init__(self)

    def run(self):
        # Continuosly runs, obtaining frames from the stream
        global drone_image,count,keep_running
        # 'ffmpeg -i  rtsp://192.168.99.1/media/stream2 -async 1 -vsync 1 -vcodec rawvideo -pix_fmt rgb24 -f rawvideo -'
        cmd = 'ffmpeg -i  rtsp://192.168.99.1/media/stream2 -vcodec rawvideo -pix_fmt rgb24 -f rawvideo -'
        p1 = Popen(cmd, stdout=PIPE, shell=True)
        fd = p1.stdout
        sx,sy=640,360
        while keep_running:
            # Reads the fram at a certain size
            s = fd.read(sx * sy * 3)
            # Saves and resizes it
            img = np.fromstring(s,dtype='uint8').reshape((sy,sx,3))
            drone_image = img # Unnessecary statement
            count = count + 1

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
    
    # Start the second thread
    Obtain_Image_Thread = image_thread()
    Obtain_Image_Thread.start()

while (1):
    # grab the latest frame, if any
    if drone_image is not None:
        cv2.imshow('image', drone_image)
        drone_image=None
        #print(count)
    else:
        # Frame has either not been captured or missed
        print("not working")
    mambo.smart_sleep(0.1)
    #time.sleep(0.5) # This is bad to use sleep with Mambo
    if cv2.waitKey(1) & 0xFF == ord('q'):
        keep_running=False
        break

Obtain_Image_Thread.join()
cv2.destroyAllWindows()













