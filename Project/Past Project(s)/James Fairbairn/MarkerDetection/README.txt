This project requires OpenCV 3.1 with the additional module Aruco. It is also necessary to have to RealSense SDK installed for working with the F200 and R200 cameras.

General Notes:
- For use with the different cameras there are two sections of code. Currently it is set up to work with the R200 camera. To change this comment out line 401 of main.cpp and uncomment the lines below it. This could probably be merged into one thing in the StreamManager class but has not been done as of yet.
- The natural feature tracking aspect of the project requires a reference image. Currently it is set up in such a way that it will generate a reference image from the first few frames of the camera. If this is to be changed see line 410 onwards of main.cpp
- If you wish to change the Aruco marker to one other than that provided (marker.jpg) then there is a site online for generating them http://keystone.umd.edu/html/markergen.html. For further information reference http://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html#gsc.tab=0. Note that if the dimensions of the marker change the dictionary used for matching will need to be changed (line 429 of main.cpp)

Overview of Process

The way in which the program works is:

- Firstly it sets up the camera as necessary and retrieves the appropriate feeds from it (usually depth and colour). During this phase it also sets up a reference image to be used with the natural feature tracking.
- It then starts to iterate through live frames from the camera. It looks for markers in the frame while also running a NFT search in the background.
- If it finds markers in the image it determines the direction necessary to move to centralise the marker.
- If the marker is roughly central it will draw "Go Down on the Screen" as long as it is above 50 cm off the ground. It uses some depth history and averaging to calculate the distance to the ground as the range of the cameras is very limited.
- If we haven't detected a marker it will try to use natural feature tracking for the directional movement. As natural feature tracking is not overly accurate it checks to see if 2 or more of the last 10 frames have contained a marker. If they have it doesn't do anything. If not it will use NFT. It calculates a homography matrix and attempts to determine the direction of movement from this. It's not overly accurate though.

This is the basic outline of how the program works. The majority of what is going on in the main function is briefly commented for clarity.