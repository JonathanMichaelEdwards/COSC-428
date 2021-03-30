#!/bin/bash

# Download the repository.
git clone https://github.com/xl-sr/THOR
cd THOR
git checkout 0a36b56ff5b1435c4c5d10c6a6d0614ff42bde07 

# Download the CNN models if they don't already exist.
cd 'trackers/SiamRPN'
if [ ! -f "model.pth" ] 
then
    filename="SiamRPNBIG.model"
    fileid="1-vNVZxfbIplXHrqMHiJJYWXYWsOIvGsf"
    wget --save-cookies cookies.txt 'https://docs.google.com/uc?export=download&id='${fileid} -O- \
        | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt

    wget --load-cookies cookies.txt -O 'model.pth' \
        'https://docs.google.com/uc?export=download&id='${fileid}'&confirm='$(<confirm.txt)
fi

cd '../SiamMask'

if [ ! -f "model.pth" ] 
then
    wget http://www.robots.ox.ac.uk/~qwang/SiamMask_VOT.pth -O 'model.pth'
fi
cd ../..

# Run the webcam demo
python3 webcam_demo.py --tracker SiamRPN
