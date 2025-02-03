#!/bin/bash

source /opt/ros/noetic/setup.bash
cd ~/opendr
source bin/activate.sh
cd projects/opendr_ws
source devel/setup.bash
cd ~/ROBO-666/corob_2024/ros1env
source devel/setup.bash
cd ~/ROBO-666/bash_scripts
roslaunch chatter_server chatter.launch
