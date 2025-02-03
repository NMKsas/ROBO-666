# Fusion control - package

This package contains control server structures to respond requests sent by behavior tree client in `fusion` package. 

Currently different ROS1 action servers are launched in file `fusion_control_node`. The action servers are built upon `MoveItActionWithSteps` abstract class, and distinguishable by their postfix `_action_srv.py`. 

The actual control is handled in the following files using mainly `MoveIt!` -library - all the honor for these clean interface files go to Ossi Parikka, [GitHub - ozzyuni/LMPVC](https://github.com/ozzyuni/LMPVC): 

- `controller_cli.py`
- `controller_srv.py`
- `controller.py`
- `grasp_client.py`

## Requiremenets

Besides the regular ROS1 dependencies, you will need 

- MoveIt! library 
- franka gripper library 

Run `rosdep install` command in the workspace top directory to install the dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Launch

After building the package, you should be able to launch the necessary nodes with 

```bash
roslaunch fusion_control control.launch 
```

Modify the `fusion_control_node` accordingly, to decide which action servers should be launched. Currently the following actions have been verified to work (more or less): 

- Pick.action 
- Place.action
- Give.action
- Move.action 

## Simulation on Rviz

If you want to simulate the control node on Rviz, you can use the demo launch that should come with the MoveIt! library;  

```bash
# rviz with the panda robot should appear 
roslaunch panda_moveit_config demo.launch
```

And launch the fusion_control on second terminal  

```bash
roslaunch fusion_control control.launch 
```

When simulating the robot in Rviz, comment out the gripper control such as controller_cli.open_hand(), or else the simulation can't proceed.
