# d435_segmentation.py

# Make Python look for the right place for pyrealsense2 for the COSC lab PCs.
import sys
sys.path.append('/usr/local/lib') 

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2

path_to_bag = "rgbd_segmentation.bag"  # Location of input file.

config = rs.config()
# This specifies that we are loading pre-recorded data 
# rather than using a live camera.
config.enable_device_from_file(path_to_bag)

pipeline = rs.pipeline()  # Create a pipeline
profile = pipeline.start(config)  # Start streaming
# Saving to a .bag file is done before alignment, so we need to do it when
# reading from .bag files too.
align = rs.align(rs.stream.color)

depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
print("Depth Scale is: {:.4f}m".format(depth_scale))

while True: # Loop over the images until we decide to quit.
    # Get the next frameset.
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Extract the frame images.
    depth_image = np.asanyarray(aligned_frames.get_depth_frame().get_data())
    color_image = np.asanyarray(aligned_frames.get_color_frame().get_data())

    # Create a new 8-bit colour version of the depth data for drawing on later.
    # This is scaled for visibility (65536 / 3500 ~= 18)
    depth_frame_scaled = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)

    # Do a simple "in range" threshold to find all objects between 2 and 5 units of distance away.
    # Note that each increment is equal to approximately 256*depths_scale. This is because the 
    # inRange function only accepts 8-bit integers, so we must scale it down.
    thresh = cv2.inRange((depth_image / 256).astype(np.uint8), 2, 5)

    # Perform some morphological operations to help distinguish the features in the image.
    kernel = np.ones((3,3), np.uint8)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=4)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel, iterations=4)

    # Detect the contour in the image.
    contours, hierarchy = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw the contour around the detected object.
    cv2.drawContours(color_image, contours, -1, (0,0,255), 1)
    cv2.drawContours(depth_frame_scaled, contours, -1, (0,0,255), 1)
    # Use the moments of the contour to draw a dot at the centroid of the object.
    for contour in contours:
        moments = cv2.moments(contour)
        cX = int(moments["m10"] / moments["m00"])
        cY = int(moments["m01"] / moments["m00"])
        # Draw the centroid on the colour and depth images.
        cv2.circle(color_image, (cX, cY), 7, (0, 255, 0), -1)
        cv2.circle(depth_frame_scaled, (cX, cY), 7, (0, 255, 0), -1)

    cv2.imshow('depth', depth_image)
    cv2.imshow('color', color_image)

    if cv2.waitKey(30) & 0xFF == ord('q'):
        break
