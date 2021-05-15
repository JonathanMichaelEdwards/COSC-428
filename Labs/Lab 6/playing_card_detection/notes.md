# The Playing Card Neural Network Example

Here's an example of "cheating" to generate a lot of examples to train and validate our neural network against. For this example, we're looking at training a neural network so that it will detect playing cards based on the information in the top left and bottom right of the cards. We're going to do this by producing card images, and automatically annotating them, and then using a number of backgrounds from the [DTD](https://www.robots.ox.ac.uk/~vgg/data/dtd/) project to generate a number of "hands".

From there, we can train the YOLOv3 detector on these generated images and compare them against real-world examples.

This example builds upon the work made available from the [playing-card-detection git repo](https://github.com/geaxgx/playing-card-detection) and an [alternative Darknet implementation](https://github.com/eriklindernoren/PyTorch-YOLOv3) running on PyTorch.

## Getting Started
1. Firstly, download the backgrounds dataset from [here](https://www.robots.ox.ac.uk/~vgg/data/dtd/download/dtd-r1.0.1.tar.gz) and extract it into the playing_cards directory.
2. Then, from within the directory, execute `python3 generate_card_data.py`. This will generate the training and validation images.
3. Convert the generated Pascal VOC annotation into something we can use with YOLOv3 by running `python3 convert_voc_yolo.py data/scenes/val data/cards.names data/valid.txt` and `python3 convert_voc_yolo.py data/scenes/train data/cards.names data/train.txt`
3. Convert the generated Pascal VOC annotation into something we can use with YOLOv3 by running 
`python3 convert_voc_yolo.py data/custom/valid data/cards.names data/valid.txt` and 
`python3 convert_voc_yolo.py data/custom/train data/cards.names data/train.txt`

## Alternate Darknet Install (PyTorch)
1. `pip3 install tensorflow tensorboard terminaltables pillow`
1. `git clone https://github.com/eriklindernoren/PyTorch-YOLOv3.git`
1. `cd PyTorch-YOLOv3 && git checkout 47b7c912877ca69db35b8af3a38d6522681b3bb3`
2. Download the pretrained network `cd weights && wget -c https://pjreddie.com/media/files/darknet53.conv.74`
3. Create the custom model `cd ../config && bash create_custom_model.sh 52`
4. Copy the contents of the `cards.names` file into `/data/custom/classes.names`.
4. Copy all of the generated .jpg files from your train and val directory into `/data/custom/images/`.
5. Copy all of the generated .txt files from your train and val directory into `/data/custom/labels/`.
6. Copy the generated `train.txt` file over to `/data/custom` and do a search on `train` and replace for `images` to update the directory path.
7. Similarly, do the same for `valid.txt`, doing the search for `valid` and replacing it with `images`.
8. Train the network `python3 train.py --model_def config/yolov3-custom.cfg --data_config config/custom.data --pretrained_weights weights/darknet53.conv.74 --batch_size=6 --checkpoint_interval=10 --epochs=1000` (Runs for about 12 hours on a nVidia 1080Ti)

Things to note on the training: 

Once your network is training, you can run `tensorboard --logdir='logs' --port=6006` in a terminal running from the root of the git repository. If you then visit `localhost:6006` in your browser you will get a real-time dashboard of the progress of your training. This can be really important as more training doesn't always make for better results as can be seen from the below graph:
![](./val_mAP.svg)
The x-axis is time, and the y-axis is the mAP value of the network as training progressed (mAP stands for "Mean Average Precision", which is the average accuracy over all classes for a network). As you can see, towards the end, there's a major drop of ~20%. This is also the reason why we take checkpoints of the network during the training process. The checkpoints are simply the network at the given timestep, and thus we can select the best network from the training process.

## Inference
python3 detect.py --image_folder data/samples --model_def config/yolov3-custom.cfg --class_path data/custom/classes.names --weights_path ./checkpoints/trained_red.pth

## Misc Notes
Currently, the generate_card_data.py script generates 400 images each for training and validation. These numbers were chosen fairly arbitrarily. It is left to the user to figure out appropriate image counts for their particular use case. Similarly for the training parameters on the neural network.

https://medium.com/udacity-pytorch-challengers/saving-loading-your-model-in-pytorch-741b80daf3c