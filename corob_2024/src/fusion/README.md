# Fusion - package

**Note**: this package is not yet fully functional. Contact @nmksas for details. 

This package contains Behavior tree structure (using [BehaviorTree.CPP 3.8)](https://www.behaviortree.dev/) in C++ for orchestrating the main robot actions, such as Pick and Place jobs. Currently the code is arranged in subdirectories as follows: 

| directory                 | content                                                                                                                                                                  |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| /action                   | ROS1 action files                                                                                                                                                        |
| /include/behaviortree_ros | Wrappers for action/service nodes                                                                                                                                        |
| /include/fusion_tree      | All the fusion tree related files. Separate node classes are organized in corresponding subfolders, while the main class for the tree is defined in fusion_tree.h / .cpp |
| /launch                   | ROS1 launch files to launch the tree                                                                                                                                     |
| /src                      | The main executable                                                                                                                                                      |
| /srv                      | ROS1 service files                                                                                                                                                       |
| /test                     | Some simple tests written using gtest                                                                                                                                    |
| /tree_files               | .xml files to construct the tree.                                                                                                                                        |

## Requirements

Besides the more common ROS1 stuff, you will need: 

- BehaviorTree.CPP 3.8 library (as mentioned in CMakeLists.txt) 
- ROS1 actionlib 

After building the package in your workspace, you should be able to launch the tree by running 

```bash
roslaunch fusion fusion_bt.launch 
```

Note that the tree is still under development, and not published in the scope of ROBO.666 project. 