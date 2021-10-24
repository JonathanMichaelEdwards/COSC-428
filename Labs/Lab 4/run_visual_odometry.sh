#!/bin/bash

# Check if the orb_slam2 directory exists. If so, don't bother running the setup again.
if [ ! -d "visual_odometry" ]
then
    # Create a directory for storing all our files in.
    mkdir visual_odometry
    cd visual_odometry

    # Download the necessary files.
    git clone https://eng-git.canterbury.ac.nz/sds53/cosc428-vo-example.git
    wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz
    wget https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/associate.py

    # Extract the compressed files.
    tar -xf rgbd_dataset_freiburg1_desk.tgz

    # Run the association script on the colour and depth images from the dataset and store the output in associations.txt.
    # It is important that we use Python2.x here. Not Python3.
    python associate.py rgbd_dataset_freiburg1_desk/rgb.txt rgbd_dataset_freiburg1_desk/depth.txt > rgbd_dataset_freiburg1_desk/associations.txt
else
    # If the setup has already been run, then just move to the visual_odometry directory in preparation for running the visual odometry algorithm.
    cd visual_odometry
fi

# Run the ORB_SLAM2 algorithm on the dataset we just downloaded.

python3 ./cosc428-vo-example/vo.py rgbd_dataset_freiburg1_desk ./results.txt ./cosc428-vo-example/cfg/tum/fr1.yaml

# Once we're done, return to the original directory that we started from.
cd ..
