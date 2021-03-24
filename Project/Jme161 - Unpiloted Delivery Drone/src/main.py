#!/usr/bin/env python3

"""
    Executes the whole program
"""

from djitellopy import tello
import cv2, math, time

from camera_display import FrontEnd



# Flight controls
def controlLanding(tello):
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(3)
    tello.land()


def run(frontend, tello):
    """

    """
    frontend.streamon()  # Streaming Thread
    # while frontend.frame is None:
    #     continue
    time.sleep(1)

    print(f'\nBattery: {tello.get_battery()}\n')
    print(f'Temp: {tello.get_temp()}\n\n')

    
    time.sleep(10)
    # controlLanding(tello)

    # Kill Everything
    tello.streamoff()
    cv2.destroyAllWindows()
    quit(0)



def main():
    frontend = FrontEnd()
    run(frontend, frontend.tello)  # run frontend


if __name__ == '__main__':
    main()