# d435_to_file.py

# Make Python look for the right place for pyrealsense2 for the COSC lab PCs.
import sys
sys.path.append('/usr/local/lib') 

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2

path_to_bag = "rgbd_output.bag"  # Location of output file.

config = rs.config()

# Enable the depth and color streams. 
# The rs.format.bgr8 indicates that the colour data three 8-bit channels.
# Note the bgr (blue, green, red) format for use with OpenCV.
# The rs.format.z16 indicates that the depth data is an unsigned 16-bit integer.
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# This starts the recording process.
config.enable_record_to_file(path_to_bag)

pipeline = rs.pipeline()  # Create a pipeline
profile = pipeline.start(config)  # Start streaming
align = rs.align(rs.stream.color) # Create the alignment object.

while True:
    # Get frameset of color and depth and align the frames.
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Get aligned frames.
    depth_image = np.asanyarray(aligned_frames.get_depth_frame().get_data())
    color_image = np.asanyarray(aligned_frames.get_color_frame().get_data())

    # Show the depth and color data to the screen.
    cv2.imshow('Colour ', color_image)
    cv2.imshow('Depth', depth_image)
    
    # Close the script when q is pressed.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the camera and close the GUI windows.
pipeline.stop()
cv2.destroyAllWindows()
