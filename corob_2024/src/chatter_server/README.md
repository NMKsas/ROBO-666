# Chatter server

This package consists of two ROS node classes

- `FeedbackTalker`  - ROS1 service servers for interacting with the operator 

- `CommandListener` - subscribes a speech transcription topic to filter verified commands 

## Prerequisites and usage

The package depends on

- `audio_common` ROS package

- `opendr_perception` environment, specifically `speech_transcription_node.py`

```bash
# install audio common before use 
sudo apt-get install ros-noetic-audio-common
```

More information on speech transcription node on [OpenDR github page](https://github.com/opendr-eu/opendr/tree/master/projects/opendr_ws/src/opendr_perception). 

### 1. Launch sound node

```bash
# source and run the sound play node   
source /opt/ros/noetic/setup.bash
roscore & # if not launched yet  
rosrun sound_play soundplay_node.py 
```

### 2. Launch the chatter server nodes

```bash
source /opt/ros/noetic/setup.bash 

# source the openDR workspace 
cd /opendr
cd projects/opendr_ws/ && source devel/setup.bash 

# assuming catkin_make is already ran 
cd /ROBO-666/corob_2024/
source /ROBO-666/corob_2024/devel/setup.bash 

# launch listener, talker, speech transcription and synthesis 
roslaunch chatter_server chatter.launch 
```

## FeedbackTalker

The following ROS1 services are included in the `FeedbackTalker` class.  

| Name             | Description                                                                     | Configuration file |
| ---------------- | ------------------------------------------------------------------------------- | ------------------ |
| Confirm          | Listen to /verification topic to define, whether the user action is confirmed   |                    |
| ConfirmTarget    | Service call for confirming a target (location/object/direction)                | `targets.json`     |
| Feedback         | Use speech synthesis to utter the sent message                                  |                    |
| TaskInstructions | Based on the task id, give instructions on what needs to be done next.          | `tasks.json`       |
| IsObjectDetected | Check whether the object is seen. If no object is seen, inform the user.        |                    |
| ListTargets      | Utter the list of targets by category (location/object/direction)               | `targets.json`     |
| ListTools        | Based on the request tool mask, utter the list of needed tools for the operator | `tools.json`       |

The user has to configure the known targets as `.json` files. Examples can be found under `src/json_files`. If necessary, the class can be expanded for other use cases. 



## CommandListener

The current implementation filters the speech transcription into six different categories

| Relayed topic       | data type | Description                                                                                                 | Configuration file |
| ------------------- | --------- | ----------------------------------------------------------------------------------------------------------- | ------------------ |
| /stop_enabled       | Bool      | The stop has been enabled by speech command. The topic can be used as a triggering event for other systems. |                    |
| /verification       | Bool      | Verification yes/no for performing the task.                                                                |                    |
| /verified_command   | String    | Commands that are known by the system.                                                                      | `commands.json`    |
| /verified_object    | int16     | Objects that are known by the system.                                                                       | `targets.json`     |
| /verified_location  | int16     | Locations that are known by the system.                                                                     | `targets.json`     |
| /verified_direction | int16     | Directions that are known by the system.                                                                    | `targets.json`     |

The user has to configure the known targets as `.json` files. Examples can be found under `src/json_files`. Additionally, this release includes a `raw_speech_to_refined_commands.json` for fine-tuning, as the used speech transcription often has issues with mishearings. 








