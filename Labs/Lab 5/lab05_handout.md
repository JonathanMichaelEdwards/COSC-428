# COSC428 Lab 5 - Deep Learning for Pose and Object Recognition

## Objectives
The overall goal of this lab is to provide an introduction on how deep learning networks can be applied to computer vision problems. You will use deep learning networks to:
- estimate the optical flow between two image frames
- estimate the pose of a person from an image
- perform object recognition and segmentation to determine what objects might be present

## Preparation
Download the "Lab 5 files" zip archive from Learn, or from the [Git repository on eng-git](https://eng-git.canterbury.ac.nz/owb14/cosc428-lab5). The following files should be in that zip file:
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

## Notes
All of the code in this lab is written using PyTorch, an extension for Python that has two purposes:
1. GPU acceleration for mathematical operations as a replacement for Numpy
2. Convolutional Neural Networks (CNNs) for recognising objects

That being said, the first purpose really exists in the service of the second. 

Artificial Neural Networks (ANN) are designed to simulate the interactions between neurons (as best as we currently understand them), thus forming a rudimentary brain-like system. While the computational requirements for a CNN (deep learning network) can be quite intense, and training a CNN can require a large number of examples, the results can far exceed the effectiveness of traditional techniques. A notable example of this approach was the development of Google’s AlphaGo program for playing Go.

## Optical Flow with FlowNet2
Optical flow was demonstrated back in Lab 3 using the Lucas-Kanade method. In this lab, we will look at optical flow as calculated by the [FlowNet2 neural network](https://arxiv.org/abs/1612.01925). The first version of FlowNet simply used a single neural network to detect the displacement of pixels. The network was trained using a dataset that consists of a computer simulation of a “flying chair”. This allowed for accurate ground truth data for correcting the neural network.

For FlowNet2, several FlowNet networks were combined, each trained for large or small displacements, before the final output is produced by a “fusion” network as the last stage of the algorithm.

### To do:
- Run `bash flownet.sh` in the terminal. This takes care of downloading everything you need to get things going, and starts the script running.

The neural network outputs two single-channel (or grayscale) images containing the horizontal and vertical components of the flow image:
- For the horizontal component, positive values relate to objects moving from left to right (from the webcam's perspective), and conversely, objects moving right to left produce negative values.
- Similarly, movement from the top of the viewport to the bottom produce positive values for the vertical component, and objects moving from the bottom to the top produce negative values.

Additionally, there's a third window that uses RGB to represent both axes in the same frame. All of the scripts for this network can be found on [our fork of the original repository](https://eng-git.canterbury.ac.nz/zane.barker/flownet2-pytorch). The functions for achieving this output, the ‘flow2rgb()’ and ‘make_color_wheel()’ functions, are copied from the mmcv library, specifically optflow.py.

One key point of interest is the `read_camera()` function within the `webcam_demo.py` script. All this does is trim the image down to a size that is divisible by 64, as required by the neural network. If necessary, you can adjust exactly what size the image is trimmed to using the HORZ_PIXELS and VERT_PIXELS variables.


## Pose Estimation
Pose Estimation is the process of determining what pose a person is currently in based on some data. For example, where keypoints such as their head, hands, feet and joints are. There are a range of applications for this; the most common being for use in the film or video game industry. Until recently, the best option for those who couldn’t afford massive camera arrays and dozens of reflective dots, was to apply 3D camera technology. The most notable implementation of this method came from Microsoft with the Kinect camera on the Xbox 360.

Incredibly, we don’t even need a 3D camera anymore. Thanks to pretrained models and network, all we need to produce a shockingly good pose estimation is a simple 2D image. This is run with Facebook Research's [Detectron2 project](https://github.com/facebookresearch/detectron2).

### To do:
- Run `bash pose_estimation.sh` in the terminal. This takes care of downloading everything you need to get things going, and starts the script running.
- Observe the keypoints overlaid on the frame as you move around in the frame.

The script run here is `detectron2/demo/demo.py`. If you run the script with `--help` instead, you can see how to load in images or videos instead.

An example of leveraging Detectron2's keypoint detection will be shown in Lab 6.


## Object Recognition and Segmentation
Remember how in the last few labs we performed segmentation with colour (with varying results that are very dependent on how uniformly coloured the object is, and how consistent the lighting is), or using depth from a 3D camera? Well neural networks can help here too! 

Mask-RCNN is the latest in a line of related neural network systems that have been successfully applied to image recognition. The details and history are a bit outside the scope of this lab, but if you're so inclined, you can read about it [here](https://blog.athelas.com/a-brief-history-of-cnns-in-image-segmentation-from-r-cnn-to-mask-r-cnn-34ea83205de4).

This next section is all about how to train and run a neural network using the same Detectron2 project from Facebook Research as we used for the keypoint detection, this time with different neural network model, however. To start with, we’re just going to run the pre-trained network.

**The model for the neural network is a bit over 360MB. If you run into weird issues here, make sure you have enough space available to store the model.**

### To do:
- Run `bash detectron2.sh` in the terminal. This takes care of downloading everything you need to get things going, and starts the script running.
- Point your webcam around the room and take note of the objects that are detected.
- Play around with how little of an object can be shown to the webcam before it is detected.



## Training a Neural Network using LabelStudio
[LabelStudio](https://labelstud.io/) is a tool for annotating a range of machine learning datasets, and also has the ability to train neural networks on these datasets.

LabelStudio will work on a range of datasets including, audio, text, images, and HTML data.

For the sake of this example, we're going to attempt to train a neural network that will distinguish between Yu-Gi-Oh, Pokemon, and Magic: The Gathering cards.

### To do:
- Run `bash labelstudio.sh`.
- This will kick of a bit of background setup. Once that's done, a browers window will open up.
- Start by clicking on the blue "Import" button.
- Navigate to the "card_train" directory in the git repository, and drag the images (or some subset thereof) onto the upload icon.
- Once they're uploaded, click "Import".
- An interface of all the "tasks" (in this case images) will be loaded.
- Click the blue "Label" button on the far right to continue. You'll be asked to set up your label configuration first.
- Copy and past the following into the box below `Edit labels and classes in Labeling Config`, replacing what is already there.
```
<View>
  <Image name="image" value="$image"/>
  <RectangleLabels name="label" toName="image">
    <Label value="MTG" background="green"/>
    <Label value="Pokemon" background="blue"/>
    <Label value="Yugioh" background="red"/>
  </RectangleLabels>
</View>
```
- Click the Save button to continue.
- Once you're back on the Tasks page, click the label button again.
- You will be presented by one of the images that's been loaded in. This is your cue to start labeling.
- To start, select one of the categories below the image, and drag a box around the card in the image. Once you're done, click "Submit".
  - Don't rotate the bounding boxes. Since we're using the "RectangleLabels" class, the system can't handle anything that isn't parallel to the edges of the image. If you want to do that for your own project, you can adjust the labeling configuration to suit.
- Annotate the rest of the data.
- Once this is done, open another terminal window from the git repository and, run `bash labelstudio-ml.sh` to initialise the machine learning backend.
- In the web gui, open the "model" tab from the toolbar at the top of the screen.
- Enter the IP address from the labelstudio-ml script (probalby `http://localhost:9090`) and hit "Add Backend".
- Once the network is connected, hit "Start Training". You can watch the training progress in the terminal.
  - By default, the training is run for 30 epochs. This can be changed by modifying the default values for the `ObjectDetectorAPI.fit()` parameters in `model.py`.


### Using a Trained Network
Once a network has been trained, you can either copy out the trained model or do additional labelling, using the trained network as a helper. As the nextwork is trained, LabelStudio will offer better and better suggestions, ultimately getting to the point where, more often than not, one need not manally label their data at all. 

You can try this by loading in the data from `card_test`, and attempting to label it. You'll find that the results are all pretty terrible, but additional training data should fix that up if you cared to commit the time to it.

There are two caveats to this process at present:
 - The suggestions, or "predictions" as they are called in the GUI are a "take it or leave it" deal. You can't tweak them, you can either accept them or create your own label from scratch. 
 - The current version of LabelStudio has a bug in the GUI JavaScript where labelling after training the network can result in a `e.id.replace is not a function` error being reported in the GUI. This doesn't seem to affect the labelling process, so you can ignore it.


To extract the network from LabelStudio, you need to copy the model file from the machine learning backend's directory. The exact path is randomly generated each time, but the general format is as follows:
```
cosc428-lab5/labelstudio/ml_backend/labelstudioXXXX/YYYYYYYYYY/model.pt
```
Where XXXX is a random hexadecimal number (0-9, A-F), and YYYYYYYYYY is a random integer. If you've restarted the machine learning backend multiple times, you'll have several "labelstudioXXXX" folders. The currently active one can be seen in the Model tab of the LabelStudio web interface.

To use this model in another PyTorch project, you'll need the following lines:
```
model = models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
model.load_state_dict(torch.load(PATH_TO_MODEL.PT))
```



### Warning 
Labelstudio is still pre v1.0 software. And as such, there are some minor issues with its use. The most significant of this is how the model is stored and referenced. If you train a model and then stop the label-studio-ml process, the reference to the trained model is lost. This is not a major problem most of the time, since one tends to train a model, and then copy use it elsewhere, but it's still worth being aware of. Especially if you want to use a semi-trained model for helping with further annotatons.


### LabelStudio Troubleshooting
#### Out of Memory Issues
A common problem with training neural networks is running out of memory. If this happens, you can reduce the batch size and try again. This is done by adjusting the default “batch_size” parameter in the definition for the `ObjectDetectorAPI.fit()` method contained in the `model.py` script.

#### Super slow training/CUDA not available
This is a bit niche, but for whatever reason, if you have suspended your computer, it is known that CUDA will stop being available to Python. This can only be fixed by restarting the computer.

#### Changes to model.py
If you do make changes, make sure you modify the version inside the `ml_backend/`, or else your changes won't affect the training process. Also, any changes to `model.py` require the machine learning backend to be restarted for those changes to be detected.