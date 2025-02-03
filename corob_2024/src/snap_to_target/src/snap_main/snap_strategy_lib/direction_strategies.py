#!/usr/bin/env python3
from snap_strategy_lib.snap_action_node import SnapStrategy
import rospy 
from geometry_msgs.msg import PoseWithCovariance 
from std_msgs.msg import Int16 

class SpeechDirectionStrategy(SnapStrategy): 
    """
    Speech based strategy. Listens to verified directions published by speech 
    transcription.
    """
    def __init__(self): 
        super().__init__() 
        self._verified_direction_sub = rospy.Subscriber("/verified_direction", 
                                                         Int16, 
                                                         self.target_callback)
        self._verified_direction = None

    def target_callback(self, msg): 
        self._verified_direction = msg.data

    def reset(self): 
        self._verified_direction = None 

    def snap_to_target(self): 

        target_id = None 
        target_pose = None 

        if self._verified_direction is not None: 
            
            target_id = self._verified_direction
            target_pose = PoseWithCovariance()
            target_pose.pose.position.x = 0
            target_pose.pose.position.y = 0
            target_pose.pose.position.z = 0
            target_pose.pose.orientation.x = 1 
            target_pose.pose.orientation.y = 0
            target_pose.pose.orientation.z = 0
            target_pose.pose.orientation.w = 0

        return target_id, target_pose 
