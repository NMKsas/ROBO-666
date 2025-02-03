#!/usr/bin/env python3

"""
Constant values that are used inside different scripts of the object detection system. 

"""

### Confidence  threshold for object detection model ###
CONFIDENCE = 0.5

### Paths depending on the computer being used ###
# RoboLab computer

ST_OBJECT_DETECTION_WEIGHTS = "<your_local_path_to_ROBO-666_parent_directory>/ROBO-666/object_detection/object_detection_ws/yolo/yolov5/runs/train/exp14/weights/best.pt"
EE_OBJECT_DETECTION_WEIGHTS = "<your_local_path_to_ROBO-666_parent_directory>/ROBO-666/object_detection/object_detection_ws/yolo/yolov5/runs/train/exp3/weights/best.pt"
LOCAL_MODEL_PATH = "<your_local_path_to_ROBO-666_parent_directory>/ROBO-666/object_detection/object_detection_ws/yolo/yolov5"
TARGETS_PATH = "<your_local_path_to_ROBO-666_parent_directory>/ROBO-666/corob_2024/ros1env/src/json_files/targets.json"