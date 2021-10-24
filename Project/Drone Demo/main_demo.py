#!/usr/bin/env python3

from easytello import tello

# from face_detector import FaceDetector

import cv2
import numpy as np

import time
import threading


face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

def detect_face(img):
    smaller = cv2.pyrDown(img)
    face_img = smaller.copy()
    face_rectange = face_cascade.detectMultiScale(face_img)
    for (x,y,w,h) in face_rectange:
        cv2.rectangle(face_img,(x,y),(x+w,y+h),(30,255,30),2)
    cv2.imshow('Video Face Detect', face_img)
    return face_img


class FrontEnd(object):

    def __init__(self):
        self.iteration = 0
        self.tello = tello.Tello() # Tello object for drone interaction
        # self.detector = FaceDetector() #FaceDetection()
        self.frame = None
        self.faces = None

    def streamon(self):
        self.tello.send_command('streamon') # Send command to turn on video stream from tello drone
        video_thread = threading.Thread(target=self._video_thread)
        video_thread_daemon = True
        video_thread.start()

    def _video_thread(self):
        cap = cv2.VideoCapture('udp://' + self.tello.tello_ip + ':11111')
        while True:
            ret, self.frame = cap.read()
            time.sleep(1/60000) # pause for frames
            #cv2.imshow('Video Stream', cv2.pyrDown(self.frame))
            self.faces = detect_face(self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                exit(0)


    def run(self):

        self.streamon()
        while self.frame is None:
            continue
        time.sleep(1)

        scanning = True

        print(f'\nBattery: {self.tello.get_battery()}\n')
        print(f'Temp: {self.tello.get_temp()}\n\n')

        self.tello.takeoff()
        yaw = int(self.tello.get_attitude()[2])
        self.tello.up(0.5)

        start_orientation = yaw

        time.sleep(0.2) # just to know where i am
        print('\nSleep: A\n')

        should_stop = True
        should_stop = False
        counter = 0

        flight = True

        while not should_stop:

            if flight:
                self.tello.cw(90)
                self.tello.forward(25)

                # self.tello.ccw(90)
                # self.tello.forward(500)

                # self.tello.ccw(180)
                # self.tello.set_speed(100)
                # self.tello.right(1000)

                # self.tello.ccw(90)
                # self.tello.right(500)

                # self.tello.ccw(90)
                # self.tello.right(500)

            else:
                self.tello.down(0.2)
                self.tello.up(0.2)
                self.tello.ccw(360)


            time.sleep(1)
            print("Time to land")
            time.sleep(1)

            should_stop = True

        self.tello.land()
        self.tello.send_command('streamoff')

        cv2.destroyAllWindows()
        quit(0)


def main():
    frontend = FrontEnd()
    frontend.run()  # run frontend


if __name__ == '__main__':
    main()
