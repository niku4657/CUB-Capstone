# Augmented Guidance Using Perception Sensors

Copyright (C) 2022 Trimble

## Description

Used to read a video and publish sensor image message 
for testing detection nodes. Publishes every 1ms

ROS2 Galactic

## Publications & Subscriptions

Pub/Sub | Type | Topic Name | Description

Pub | sensor_msgs::msg::Image | perception_image | Publishes image for testing detection nodes

## Configurable Parameters

Parameter | Type | Default | Description

in_str | string | None | input path for video to read in

start_frame | int | None | first frame of video to publish

end_frame | int | None | last frame of video to publisher, enter negative value to publish entire video

frame_step | int | None | number of frames to skip before publishing next frame

## Usage

To launch the node standalone with no additional nodes, source the workspace and run:

ros2 run row_detection test_publisher --ros-args -p in_str:=InputVideoPath -p start_frame:=FristFrameToRead -p end_frame:=LastFrameToRead -p frame_step:=FrameStep

### Minimal working example

To read the video "ex.mp4" and publish every frame of video

ros2 run row_detection test_publisher --ros-args -p in_str:="CURRENT_WD/ex.mp4" -p start_frame:=0 -p end_frame:=-1 -p frame_step:=0

### Any other initial setup

**Requires** cv_bridge

**Requires** OPENCV

**Requires** sensor_msgs
