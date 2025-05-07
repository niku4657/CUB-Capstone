# Augmented Guidence Using Perception Sensors - Trimble Autonomous CSCI Senior Capstone 2021-2022

### Recreating EXPO Results
#### Recreating ML videos from EXPO
The videos found in the google drive in /Videos/"ML Results Videos"/ were created with trimble-ml/ML_Row_Detection.ipynb. Detailed commentary is provived in this notebook, please refer there to recreate the ML videos displayed at the project expo.

#### Recreating ROS videos from EXPO
The videos found in the google drive in /Videos/"CV/DL Output Videos"/ can be recreated with the following commands:

``` Bash
# for details on running each ros node and parameter information please see READMEs in row_detection/src 
source ros install
source workspace setup
# replace InputVideoPath with a path to one of the videos provided by Trimble for testing
ros2 run row_detection test_publisher --ros-args -p in_str:=InputVideoPath -p start_frame:=FristFrameToRead -p end_frame:=LastFrameToRead -p frame_step:=FrameStep
# to recreate OutVid1-4
ros2 run row_detection cv_row_detection --ros-args -p out_str:="CvOutputVideo.mp4" -p write_vid:=true
# to recreate Outvid5-6, after installing model
ros2 run row_detection dl_row_detection --ros-args -p out_str:="DlOutputVideo.mp4" -p write_vid:=true
```

### row_detection
This is the ROS package we developed with nodes to run both our classical and deep learning algorithms. Additionally there is a publisher node for easy testing 
of nodes. More documnetation for the package can be found within. To run the package, after following the install and setup instructions found [here](https://docs.google.com/document/d/1IvrRoerWze33tCi0YS-Z0yw_e3JxBHMlJ7be6PJWC5w/edit?usp=sharing), follow these instructions

``` Bash
source ros install 
source workspace setup
colcon build --pacakges-select row_detection
#start publishing 
ros2 run row_detection test_publisher --ros-args -p in_str:=InputVideoPath -p start_frame:=FristFrameToRead -p end_frame:=LastFrameToRead -p frame_step:=FrameStep
#start detection nodes and save predictions to video
ros2 run row_detection cv_row_detection --ros-args -p out_str:="CvOutputVideo.mp4" -p write_vid:=true
ros2 run row_detection dl_row_detection --ros-args -p out_str:="DlOutputVideo.mp4" -p write_vid:=true
```

### trimble-ml
This contains the machine learning notebook used to create several videos showcased at the project expo. ML_Row_Detection.ipynb contains methods and documentaion for the methods used. The methods in ML_Row_Detection were some of the **most** **promising** methods we encountered.

### classical_computer_vision
Here you will find research and the various methods we developed for row detection using classical computer vision. More documentaion can be found in this directory.

### deep-learning-annotations
Annotations we used for training our deep learning model.

### exPointDetection.tar.gz
From Trimble, this is the point detection foundation we used for our deep learning model. 
