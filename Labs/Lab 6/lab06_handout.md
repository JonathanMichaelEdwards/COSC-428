# COSC428 Lab 6 - Deep Learning for Object Tracking

## Objectives
This further develops deep learning for computer vision to train deep learning networks and apply them to tracking objects.
In this lab you will: 
- Use the "THOR" project to track generic objects without special CNN training.
- Programmatically generate training data to make up for a lack of read-world data and train a network using that data using YOLOv3.
- Look at an example project where pose tracking and colour segmentation is used to track juggling performance.

## Preparation
Download the "Lab 6 files" zip archive from Learn, or from the [Git repository on eng-git](https://eng-git.canterbury.ac.nz/owb14/cosc428-lab6). The following files should be in that zip file:
- card_test/
- card_train/
- labelstudio/
- demo_e2e_mask_rcnn_X_101_32x8d_FPN_1x.png
- detectron2.sh
- flownet2.sh
- lab05_handout.md
- labelstudio.sh
- labelstudio-ml.sh
- post_estimation.sh
- pose_single_image.py
- pose_webcam.py

## Troubleshooting
- If you get a video I/O error, change the “0” in VideoCapture to “-1” which grabs the first available camera.
- If you still cannot access the camera at any time, try unplugging it and plugging it back in.
- You can check your camera exists on Linux with: `ls /dev/video*` in the bash terminal.
- Each of the activities in this lab requires a lot of space for the neural network models. If you’re running into weird issues when downloading a file, make sure you have enough space available.


## Object Tracking with THOR
Tracking an object as it moves through a scene is high on the list of things we might want to do in computer vision. One of the major issues, however, is that the traditional method requires being able to recognise that object first. For example, earlier in the lab series, we tracked a ball using colour segmentation. In the case of more complicated objects, we’ve also looked at depth segmentation and using neural networks that have been trained to detect a class of object. The traditional neural network approach requires lots of data in advance, as well as the time required to train the network for the desired class of object.

[THOR](https://github.com/xl-sr/THOR) approaches this problem another way. Instead, a subset of a frame from a stream is selected and loaded into a buffer. As the stream continues, a neural network is applied to the task of locating the object in the frame and updating the buffer with new template images. Instead of using a single buffer, THOR has two, described as the short-term and long-term buffers. The idea is that the short-term buffer takes care of small changes in the tracked object, whereas the long-term buffer allows for keeping track of the object even when the object is rotated and otherwise completely unrecognisable from the originally selected frame.

### To do:
- Run `bash thor.sh`
- You can then select the object of interest by clicking and dragging on the resulting window. Try moving it around and see how the tracking algorithm follows it around the frame despite having no prior training on the object of interest. It’s not perfect, but its performance is nonetheless impressive.


## Generating Training Images and Training a Darknet Network with YOLOv3
One of the major pain points for making use of a neural network to detect an object is that it requires a lot of data. Especially if you’re wanting to effectively detect said object over a range of conditions (lighting, background, etc.). 

Here's an example of "cheating" to generate a lot of examples to train and validate our neural network against. For this example, we're looking at training a neural network so that it will detect playing cards based on the information in the top left and bottom right of the cards. We're going to do this by producing card images, and automatically annotating them, and then using a number of backgrounds from the [DTD](https://www.robots.ox.ac.uk/~vgg/data/dtd/) project to generate a number of "hands". From there, we can train and validate our network against these images with a view to having something that can work on real world data as well.

This example builds upon the work made available from the [playing-card-detection git repository](https://github.com/geaxgx/playing-card-detection) and an [alternative Darknet/YOLOv3 implementation](https://github.com/eriklindernoren/PyTorch-YOLOv3) running on PyTorch for the sake of convenience. Additionally, the playing card images themselves were obtained from the [Open Source Vector Playing Cards](https://totalnonsense.com/open-source-vector-playing-cards/) image set.

Note that the primary point of this example is to demonstrate generating data for training. If you want to actually train a neural network of your own, you're probably better off using LabelStudio from Lab 5.

### To do:
- Run `bash yolo.sh`. This takes care of all the setup required, and starts the training process. 
- While the training is running, open a terminal window in the PyTorch-YOLOv3 directory and run `tensorboard --logdir='logs'-port=6006` in the terminal. Then navigate to `localhost:6006` in your browser to see live statistics on the current state of the network as it is being trained.
- If you're so inclined, you can download some images of playing cards into the `data/samples` directory in the PyTorch-YOLOv3 repository and run `python3 detect.py --image_folder data/samples --weights_path ./checkpoints/yolov3_ckpt_900.pth --model_def config/yolov3-custom.cfg --class_path data/custom/classes.names` from the git repository. This will run inference, putting the processed images into the "output" directory.

### Some Notes on train.py for the Curious
- The `--checkpoint_interval` flag defaults to an interval of 1, after which it generates a ~250MB copy of the neural network. It's probably worth making this number larger for the sake of your hard drive. The yolo.sh script uses a checkpoint_interval of 10.
- The `--epochs` flag determines the number of times we loop over our dataset. Generally speaking, the bigger this number, the better your resultant network will be (albeit with diminishing returns), but at the cost of time required.
- The `--batch_size` flag groups images together in a single run. Every epoch involves processing each image, so by running several images in a single batch, it speeds things up by reducing the number of runs in a single epoch. In exchange, more memory is used on your graphics card. If you run into memory issues, try reducing this number. It's been shown that larger batch sizes cause issues with training, but this doesn't seem to matter too much until you get to a batch size of 100 or more.

### Additional Challenges for Those Who Want to Train Using YOLOv3.
As it currently stands, it’s necessary to train the network all in one go. There’s no support for pausing the training process and starting it up again later. This is left to the user to figure out on their own. The information in this link should get you most of the way, however.
Also, the number of training and validation images was selected arbitrarily at 400 images each. It’s left up to the user to determine if this is enough, too few, or too many. Same goes for the batch size and epoch count.



## Juggling Detector using Detectron2 for the Pose Detection
In addition to the pose detection project used in lab 5, Detectron2 will also load in pose detection neural networks to. This example shows using colour segmentation to detect some coloured balls and the keypoints of the body detected by the neural network to determine whether or not a juggler has dropped a ball.

The output video shows the keypoints overlaid on the body, as well as the location of the balls (as shown by three circles, when the balls can be detected). The colour segmentation is performed in the HSV colour space for simplicity. The top left corner of the screen shows three boolean values for whether a ball has been dropped or not (in order of red, green, orange).

Note that this project is making some very large assumptions and leaves a lot of room for improvement. 

### To do:
- Run `bash juggling.sh`. 
- Observe that when the green ball is dropped, that it's detected. This is done by comparing the Y position of the wrists with the Y position of the ball and concluding that a drop has occurred if a ball drops below both wrists.


### Some Notes on Doing This Yourself
This project was done by modifying the `predictor.py` to add `juggling_check()` in as part of the `run_on_video()` method. Amongst other things, the keypoint names can all be found in `detectron2/detectron2/data/datasets/builtin_meta.py` in the `COCO_PERSON_KEYPOINT_NAMES` variable:
- nose
- left_eye
- right_eye
- left_ear
- right_ear
- left_shoulder
- right_shoulder
- left_elbow
- right_elbow
- left_wrist
- right_wrist
- left_hip
- right_hip
- left_knee
- right_knee
- left_ankle
- right_ankle
