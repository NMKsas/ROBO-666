# Collaborative Workspace

This workspace includes the following packages 

| Package          | Description                                                      |
| ---------------- | ---------------------------------------------------------------- |
| `chatter_server` | Speech module for interpreting user commands and giving feedback |
| `fusion`         | The main logic for the collaborative application                 |
| `fusion_control` | ROS1 server for control interface. Implemented using `MoveIt!`.  |
| `snap_to_target` | ROS1 servers for performing target selection                     |
| `task_director`  | ROS1 server for monitoring the assembly pipeline.                |

Build and source the workspace to access the implemented nodes. 

```bash
# source ROS1 
source /opt/ros/noetic/setup.bash

# for opendr dependencies
cd ~/opendr
source bin/activate.sh
cd projects/opendr_ws
source devel/setup.bash

# build and source the environment
cd ~/ROBO-666/corob_2024/ros1env
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

More detailed instructions on how-to-use are included within each package's `README.md` file.






