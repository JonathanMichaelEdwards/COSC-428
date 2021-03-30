#!/bin/bash

if [ ! -d "labelstudio-env" ]
then
    echo "Can't find labelstudio-env. Exiting."
    exit 1
else
    echo "Activating Python environment."
    source "labelstudio-env/bin/activate"
fi

if [ ! -d "labelstudio" ]
then
    echo "Can't find the labelstudio directory. Are you inside the lab5 directory?"
    exit 1
else
    cd "labelstudio"
    if [ ! -d "ml_backend" ]
    then
        label-studio-ml init ml_backend
        cp "./coco_eval.py" "./ml_backend/"
        cp "./coco_utils.py" "./ml_backend/"
        cp "./engine.py" "./ml_backend/"
        cp "./transforms.py" "./ml_backend/"
        cp "./utils.py" "./ml_backend/"
    fi
    label-studio-ml start ml_backend
fi