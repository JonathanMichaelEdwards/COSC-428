Title: Fiducial marker tracking with a Parrot Mambo Minidrone
Author: Daniel de Gouw
Date: 30/05/19

This project used the Parrot Mambo drone to detect, follow and orientate itself with fiducial markers. The markers were printed out at a larger size on A4 to improve the accuracy of the algorithm.

There is a git repositry for the Parrot Mambo, which contains various demo codes and other source files. This repository is very usefull when learning how the Parrot Mambo works.
You may need to update the repository through git, where the master is located at:   /Pyparrot Source Code

The algorithms, excluding the flight commands, were initially tested on a computer for simplicity and because the drone had a short flight time of 8 minutes. I used my laptop camera to detect 
fiducial markers and output a GUI debug screen on the current frames. This screen explained what stage the algorithms were in and what they intended to do. It also provided me with the angles, 
distances and orientation of fiducial markers. The code was used to intially test the tracking algorithm before flight commands were introduced.
These files can be located at: /Algorithm Test

The stream from the drone to the computer was tested with code located in: /Flight & Stream Test 
Two approaches were used, the first was using the VLC module like the demo code did. This was unsuccessfull because it was hard to grab the frames without heavily modifying the source code.
The second was using FFmpeg to capture the stream. It was discovered that multiple threads were required for this to be successful. Unfortunately, the best performance I could achieve through 
this method led to 1.56 frames per second (Extremely Low). This slowed down my algorithm, however it still worked. I would recommend the VLC module to be modified so that the frames are sent directly
to the algorithm, or utilise the location where VLC saves each frame.... I hit some issues with reading these frames. 