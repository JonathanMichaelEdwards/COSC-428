# d435_streams.py

# Make Python look for the right place for pyrealsense2 for the COSC lab PCs.
import sys
sys.path.append('/usr/local/lib') 

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2

# Configure video streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

# Start streaming
profile = pipeline.start(config)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
print("Depth Scale is: {:.4f}m".format(depth_scale))

while True:
    # Wait for a coherent set of frames.
    frames = pipeline.wait_for_frames()

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    infra0_frame = frames.get_infrared_frame(0)
    infra1_frame = frames.get_infrared_frame(1)

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    infra0_image = np.asanyarray(infra0_frame.get_data())
    infra1_image = np.asanyarray(infra1_frame.get_data())

    # Apply colormap on depth/infrared images for visibilty (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    infra0_colormap = cv2.applyColorMap(cv2.convertScaleAbs(infra0_image, alpha=0.03), cv2.COLORMAP_JET)
    infra1_colormap = cv2.applyColorMap(cv2.convertScaleAbs(infra1_image, alpha=0.03), cv2.COLORMAP_JET)

    # Stack images horizontally
    images = np.hstack((color_image, depth_colormap, infra0_colormap, infra1_colormap))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    
    # Close the script when q is pressed.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the camera and close the GUI windows.
pipeline.stop()
cv2.destroyAllWindows()