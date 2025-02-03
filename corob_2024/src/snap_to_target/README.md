# Snap to Target

This package includes ROS1 action and service server interfaces for target selection. 

The module has one main class, `SnapActionNode`, to implement a ROS1 action server. The secondary class, `SnapStrategy`, is an abstract class definition for creating different snap to target algorithms. All the strategies must implement a method `snap_to_target()` to derive the pose of the wanted target. Each `SnapActionNode` has an instance of `SnapStrategy` and uses this particular method for a number of times to find the target. 

The package also includes a simple class `SnapServiceNode` as tool selection service. 

Currently implemented strategies are divided by the target category into separate files. The user is free to come up with some alternative algorithms besides the ones provided. 
Additionally there is a simple class `SnapServiceNode` for a simple tool selection service. 

Currently implemented strategies are divided by the target category into separate files. 

| File                      | Description                                                                         |
| ------------------------- | ----------------------------------------------------------------------------------- |
| `direction_strategies.py` | Strategies to choose a direction. Currently includes only speech based strategy.    |
| `location_strategies.py`  | Strategies to choose a location. Includes both speech and gesture based strategies. |
| `object_strategies.py`    | Strategies to choose an object. Includes both speech and gesture based strategies.  |

**Note:** Gesture based strategies are not available in the scope of this project. The gesturing tool is still under development and not provided in this workspace. 

# Running the node

The node can be ran by using a launch file after sourcing the environment:

```bash
# source the environment 
source /opt/ros/noetic/setup.bash 
source /ROBO-666/corob_2024/devel/setup.bash 

# run the node 
roslaunch snap_to_target snap_node.launch
```

The user should define the available targets and tools in `.json` -files. Example files and format can be found under `src/json_files`: `targets.json` and `tools.json`. 


