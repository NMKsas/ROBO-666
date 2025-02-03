#!/usr/bin/env python3
from snap_strategy_lib.snap_action_node import SnapStrategy
from geometry_msgs.msg import PointStamped
import rospy 
import numpy as np
from collections import deque
from geometry_msgs.msg import PoseWithCovariance 
from std_msgs.msg import Int16

TABLE_LEVEL = 0.118
WORKSPACE_LIMITS = [[0.33, 0.75],[-0.36, 0.36]] # [[x_lower, x_upper],[y_lower, y_upper]]


class GestureLocationStrategy(SnapStrategy): 
    """
    Gesture based strategy. Follow the published pointer values, snap if the 
    pointing gesture remains still enough. 

    Args:
        SnapStrategy (ABC): Strategy class defining the common interface
    """

    def __init__(self, gesture_topic="/gesture_pointer/r_pointer", 
                 buffersize=15): 
        super().__init__() 

        self._default_tf_frame = "panda_link0"
        self._pointer_buffer_x = deque(maxlen=buffersize)
        self._pointer_buffer_y = deque(maxlen=buffersize)
        self._threshold = 0.05

        self._gesture_sub = rospy.Subscriber(gesture_topic, 
                                            PointStamped, 
                                            self.gesture_callback)

    def gesture_callback(self, msg):
        """
        Fill in the gesture pointer buffer. Snapping to target is only attempted
        when the buffer has enough values. 

        Args:
            msg (geometry_msgs/PointStamped): gesture location as stamped point
        """        
        self._pointer_buffer_x.append(msg.point.x)
        self._pointer_buffer_y.append(msg.point.y)

    def is_buffer_full(self): 
        """
        Return true if the buffer is true and there are enough values to start
        computing 

        Returns:
            bool: True, if the deques are full 
        """
        return len(self._pointer_buffer_x) == self._pointer_buffer_x.maxlen and \
               len(self._pointer_buffer_y) == self._pointer_buffer_y.maxlen
    
    def reset(self): 
        """
        After successful snapping, reset the buffers 
        """
        self._pointer_buffer_x.clear()
        self._pointer_buffer_y.clear()

    def get_max_range_from_average(self): 
        """
        Calculate the average of the coordinate points, calculate the point 
        distances with respect to average point. Return the maximum distance. 

        Returns:
            float: maximum distance between the points and their average 
        """
        x_coords = np.array(self._pointer_buffer_x)
        y_coords = np.array(self._pointer_buffer_y)
        x_avg = np.mean(self._pointer_buffer_x)
        y_avg = np.mean(self._pointer_buffer_y)

        distances_to_avg = np.sqrt((x_coords - x_avg)**2 + 
                                   (y_coords - y_avg)**2)
        return np.max(distances_to_avg), x_avg, y_avg


    def snap_to_target(self): 
        """
        Snap to target using pointing gestures. 

        Returns:
            target_id: int, target id as defined in the grasp detected dict 
            target_pose: PoseWithCovariance, the pose for grasping the object
        """
        # attempt to snap if the buffer is full 
        if (self.is_buffer_full()): 
            
            max_distance, x_avg, y_avg = self.get_max_range_from_average()


            # snapping is done only if the coordinates remain static enough 
            if max_distance <= self._threshold and \
            WORKSPACE_LIMITS[0][0] < x_avg < WORKSPACE_LIMITS[0][1] and \
            WORKSPACE_LIMITS[1][0] < y_avg < WORKSPACE_LIMITS[1][1]: 

                # get currently detected objects
                target_id = 200              
                target_pose = PoseWithCovariance() 
                target_pose.pose.position.x = x_avg
                target_pose.pose.position.y = y_avg
                target_pose.pose.position.z = TABLE_LEVEL
                target_pose.pose.orientation.x = 1.0
                target_pose.pose.orientation.y = 0.0
                target_pose.pose.orientation.z = 0.0
                target_pose.pose.orientation.w = 0.0
                
                return target_id, target_pose

        return None, None 


class SpeechLocationStrategy(SnapStrategy): 
    
    """
    Speech based strategy. Listens to verified locations published by speech 
    transcription.

    Args:
        SnapStrategy (_type_): _description_
    """
    def __init__(self, location_dict): 
        super().__init__() 
        self._speech_verification_sub = rospy.Subscriber("/verified_location", 
                                                         Int16, 
                                                         self.target_callback)
        self._locations_dict = location_dict
        self._verified_location = None 
        self._default_tf_frame = "panda_link0"

    def target_callback(self, msg): 
        self._verified_location = msg.data

    def reset(self): 
        self._verified_location = None 

    def snap_to_target(self): 

        target_id = None
        target_pose = None 

        if self._verified_location is not None: 
            target_id = self._verified_location
            pose = self._locations_dict[str(target_id)]['coordinates']

            target_pose = PoseWithCovariance()
            target_pose.pose.position.x = pose[0]
            target_pose.pose.position.y = pose[1]
            target_pose.pose.position.z = pose[2]
            target_pose.pose.orientation.x = pose[3]
            target_pose.pose.orientation.y = pose[4]
            target_pose.pose.orientation.z = pose[5]
            target_pose.pose.orientation.w = pose[6]

        return target_id, target_pose 
