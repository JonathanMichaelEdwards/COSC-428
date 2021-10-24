# d435_parameters.py

# Make Python look for the right place for pyrealsense2 for the COSC lab PCs.
import sys
sys.path.append('/usr/local/lib') 

# First import the library
import pyrealsense2 as rs
import time
import json

def save_camera_parameters(advanced_mode_object, json_file_path):
    # Serialize all controls to a Json string
    parameters = json.loads(advanced_mode_object.serialize_json())
    with open(json_file_path, 'w') as json_out:
        json.dump(parameters, json_out)

def load_camera_parameters(advanced_mode_object, json_file_path):
    with open(json_file_path, 'r') as json_out:
        parameters = json.load(json_out)
    # The C++ JSON parser requires double-quotes for the json object so we need
    # to replace the single quote of the pythonic json to double-quotes
    json_string = str(parameters).replace("'", '\"')
    advanced_mode_object.load_json(json_string)

# To start with, the camera must be in "Advanced Mode".
# If this is not enabled, enable it first.
dev = rs.context().query_devices()[0]  # Assuming that there's only one camera plugged in.
advanced_mode_object = rs.rs400_advanced_mode(dev)
print("Advanced mode is", "enabled" if advanced_mode_object.is_enabled() else "disabled")

# Loop until we successfully enable advanced mode
while not advanced_mode_object.is_enabled():
    advanced_mode_object.toggle_advanced_mode(True)
    # The camera resets here, so we wait a bit before trying to connect again.
    print("Sleeping for 5 seconds...")
    time.sleep(5)
    dev = rs.context().query_devices()[0]
    advanced_mode_object = rs.rs400_advanced_mode(dev)

# Now we can read/write the camera parameters as we please.
save_camera_parameters(advanced_mode_object, 'd435_settings.json')
load_camera_parameters(advanced_mode_object, 'd435_settings.json')