# ROBO-666 - Battery assembly system
This repository contains the software files for the robotic system develpoed as a part of course ROBO-666 at Tampere University. The system is designed to help a human operator in assembly process of a hybrid car battery package. The packages/nodes responsible for the movement control and decision logic can be found in directory _corob_2024_ and the ones responsible for the object detection can be found in directory _object_detection_.

## Prerequisites
1) OS: Ubuntu 20.04

2) ROS installation: Noetic Ninjemys (Follow the instructions [here](http://wiki.ros.org/noetic/Installation/Ubuntu))

3) librealsense2 library installation (Follow the instructions [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md))

4) realsense2_camera ROS package installation (Follow the instructions [here](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy))

5) pyrealsense2 Python package installation (Follow the instructions [here](https://pypi.org/project/pyrealsense2/))

6) OpenDR toolkit installation (Follow the instructions [here](https://github.com/opendr-eu/opendr/blob/master/docs/reference/installation.md). Installing the toolkit by cloning the repository is recommended.)

7) RealSense D400-series depth camera (The system has been developed and tested using this kind of a camera, but with minor changes in the code, some other camera
might also work. You can also use a rosbag, where the video input from RealSense camera has been recorded, instead of the live image from the camera.)

## Installation


### Step 1: Clone the repository

    git clone https://github.com/JokinenL/ROBO-666.git

NOTE: The instructions below assume that

- you have OpenDR toolkit installed on your PC at location "~/opendr"
- you have ROS Noetic Ninjemys installed on your PC at location "/opt/ros/noetic"
- you have this repository cloned on your PC at location "~/ROBO-666"

If the locational paths presented above are different than the ones on your PC, modify them to suit your local setup. In ths case you also need to modify the paths declared in [bash script files](./bash_scripts) accordingly.

### Step 2: Build the workspaces
**The workspace for object detection**

    source /opt/ros/noetic/setup.bash
    cd ~/opendr
    source bin/activate.sh
    cd projects/opendr_ws
    source devel/setup.bash
    cd ~/ROBO-666/object_detection/object_detection_ws
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make

**The workspace for movement control and decision making logic**

    source /opt/ros/noetic/setup.bash
    cd ~/opendr
    source bin/activate.sh
    cd projects/opendr_ws
    source devel/setup.bash
    cd ~/ROBO-666/corob_2024/ros1env
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
