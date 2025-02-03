#!/bin/bash

source /opt/ros/noetic/setup.bash
cd ~/opendr
source bin/activate.sh
cd projects/opendr_ws
source devel/setup.bash
cd ~/ROBO-666/object_detection/object_detection_ws
source devel/setup.bash
cd ~/ROBO-666/bash_scripts
roslaunch object_detection object_detectors.launch
