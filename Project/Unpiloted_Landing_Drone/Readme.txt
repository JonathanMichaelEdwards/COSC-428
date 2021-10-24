Title: Autonomously Detect, Track and Land a UAV using a Tello drone
Fiducial Marker
Author: Jonathan Edwards
Date: 31/05/21

This project used the DJI Tello drone to detect, follow and orientate itself with fiducial markers. Many markers were printed out and an optimal size was determined to be 7cm that best allowed the algorithm to work the best.

There is a git repositry for the DJI if you would like to code your own, which contains various demo codes and other source files (this can be found in the 'others' folder. This repository is very usefull when learning how the Tello works.

The algorithms, excluding the flight commands, were initially tested on a computer for simplicity and because the drone had a short flight time of 13 minutes. I used my laptop camera to detect 
fiducial markers and output a GUI debug screen on the current frames. This screen explained what stage the algorithms were in and what they intended to do. It also provided me with the angles, 
distances and orientation of fiducial markers. The code was used to intially test the tracking algorithm before flight commands were introduced.
These files can be located at: /src/main.py

Only have to do once:
- For congfiguring -- run src/config/calibrate_camera.py
- After this two files are created, these files calibrate the camera.
- whenever you run the src/main.py file these files are pre-loaded. (so do not delete them!)

Running the code:
1. run this file '/src/main.py'
2. the data outputed from the sensors are stored in a data folder

Plotting data:
- run - src/plot_angle_data.m

Extra:
- mapping folder usues a google maps mapping service (This was just for fun)
- camerLoad.py (loads the camera calibration data -- called in src/main.py (so no need to touch))
- there is a 'mech' folder which includes the '.stl' files for printing the mirror case.
- 'images' folder contains the markers used.

note(s):
- Also note using a mirror, the image must be flipped!


Things that could be improved:
* Would of used the HeliPad - Flip image if I had the time.
* Edge detection - Starting to move into a real world scenario.

Happy Coding!!!