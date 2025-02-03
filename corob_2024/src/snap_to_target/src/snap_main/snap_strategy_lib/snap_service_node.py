#!/usr/bin/env python3
import re
import rospy 
from geometry_msgs.msg import PoseWithCovariance 
from snap_to_target.srv import SnapToTool, SnapToToolResponse
from snap_strategy_lib.detection_filter import DetectionFilter

def remove_articles(text):
    return re.sub('(a|an|the)(\s+)', '', text)

class SnapServiceNode: 
    """
    Defines the interface for snap to target action servers 
    """
    def __init__(self, detection_filter : DetectionFilter, tool_dict : dict) -> None: 

        self._detection_filter = detection_filter
        self._tool_dict = tool_dict
        self._snap_server = rospy.Service('snap_to_tool', SnapToTool, 
                                          self.find_tool)
        

        rospy.loginfo(f"Snap to tool server started")

    def find_tool(self, req) -> None: 
        """
        Snap to tool callback, to get the pose of the desired tool 

        Args:
            goal (SnapToTool): ROS1 service request. Includes the tool id.

        """
        tool_name = remove_articles(self._tool_dict[req.tool_id])
        # Service time limited to 5s 
        for i in range(10): 
            self._detection_filter.update_filtered_detections()
            pose = self._detection_filter.get_target_by_id(req.tool_id)
            if pose is not None: 
                tool_pose = PoseWithCovariance()
                tool_pose.pose = pose 
                print(tool_pose)
                return SnapToToolResponse(tool_pose, tool_name, True)
            rospy.sleep(0.5)
        return SnapToToolResponse(PoseWithCovariance(), tool_name, False)