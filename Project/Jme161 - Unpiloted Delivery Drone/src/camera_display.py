"""
    Creates a video feed thread. 
"""


from djitellopy import tello
from threading import Thread
import cv2, math, time



class FrontEnd(object):
    """

    """

    def __init__(self):
        self.iteration = 0
        self.tello = tello.Tello() # Tello object for drone interaction
        self.frame = None

    def streamon(self):
        self.tello.connect()
        self.tello.streamon()               # Send command to turn on video stream from tello drone
        video_thread = Thread(target=self._video_thread)
        video_thread_daemon = True
        video_thread.start()

    def _video_thread(self):
        cap = cv2.VideoCapture('udp://' + self.tello.tello_ip + ':11111')
        while True:
            (ret, self.frame) = cap.read()
            time.sleep(1/60000) # pause for frames
            #cv2.imshow('Video Stream', cv2.pyrDown(self.frame))
            if (cv2.waitKey(1) & 0xFF == ord('q')):
                # Land safley
                cv2.destroyAllWindows()
                exit(0)

