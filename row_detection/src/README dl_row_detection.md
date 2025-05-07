# Augmented Guidance Using Perception Sensors

Copyright (C) 2022 Trimble

## Description

Runs Deep Learning Model for Detecting Crop Rows

ROS Galactic

## Publications & Subscriptions

Pub/Sub | Type | Topic Name | Description

Sub | sensor_msgs::msg::Image | perception_image | image to run model on

## Configurable Parameters

Parameter | Type | Default | Description

out_str | string | None | Output path for image/video of prediciton

write_vid | bool | None | If true open video and write predictions to mp4, else save predictions as jpg

## Usage

To launch the node standalone with no additional nodes, source the workspace and run:

ros2 run row_detection dl_row_detection --ros-args -p out_str:=OutputPath -p write_vid:=T/F

### Minimal working example

To save predictions to a mp4 named ex.mp4

ros2 run row_detection dl_row_detection --ros-args -p out_str:="/home/cody/Desktop/ex.mp4" -p write_vid:=true

### Any other initial setup

**Requires** cv_bridge

**Requires** OPENCV

**Requires** sensor_msgs

**Requires** KeyPointDetection (DL Model)