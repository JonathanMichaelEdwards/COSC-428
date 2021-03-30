#!/bin/bash

export PATH=/usr/local/cuda/bin:"$PATH"

git clone https://eng-git.canterbury.ac.nz/zane.barker/flownet2-pytorch.git
cd flownet2-pytorch
bash ./install.sh

if [ ! -d models ]; then
     mkdir models
     cd models

     filename="FlowNet2_checkpoint.pth.tar"
     fileid="1hF8vS6YeHkx3j2pfCeQqqZGwA_PJq_Da"

     wget --save-cookies cookies.txt 'https://docs.google.com/uc?export=download&id='${fileid} -O- \
          | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt

     wget --load-cookies cookies.txt -O ${filename} \
          'https://docs.google.com/uc?export=download&id='${fileid}'&confirm='$(<confirm.txt)

     cd ..
fi

python3 webcam_demo.py