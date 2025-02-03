#!/usr/bin/env python3
import rospy 
import numpy as np
from math import sqrt 
from collections import deque
from snap_strategy_lib.detection_filter import DetectionFilter
from snap_strategy_lib.snap_action_node import SnapStrategy
from geometry_msgs.msg import PoseWithCovariance 
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int16 

DEFAULT = 100 # metres, large enough init value 

class GestureObjectStrategy(SnapStrategy):
    """
    Gesture based strategy. Follow the published pointer values, snap if the 
    pointing gesture remains still enough. 

    Args:
        SnapStrategy (ABC): Strategy class defining the common interface
    """

    def __init__(self, detection_filter : DetectionFilter, gesture_topic="/gesture_pointer/r_pointer", 
                 buffersize=15): 
        super().__init__() 
        self._detection_filter = detection_filter
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
            if max_distance > self._threshold: 
                return None, None
            else: 
                # get currently detected objects
                self._detection_filter.update_filtered_detections() 
                objects = self._detection_filter.get_filtered_detections()

                # initialize snapped target
                target_id = None                 
                target_pose = PoseWithCovariance()
                
                # initialize with large distance
                min_distance_object = DEFAULT

                for id in objects: 
                    for i in range(len(objects[id])):
                        # calculate distance to each recognized target 
                        distance = sqrt((objects[id][i].position.x - x_avg)**2
                                       +(objects[id][i].position.y - y_avg)**2)
                        
                        # choose the closest target 
                        if  distance < min_distance_object:  
                            min_distance_object = distance
                            target_id = id 
                            target_pose.pose = objects[id][i] 
                return target_id, target_pose

        return None, None 

class SpeechObjectStrategy(SnapStrategy): 
    """
    Speech based strategy. Listens to verified objects published by speech 
    transcription.
    """
    def __init__(self, detection_filter : DetectionFilter): 
        super().__init__() 
        self._detection_filter = detection_filter
        self._speech_verification_sub = rospy.Subscriber("/verified_object", 
                                                         Int16, 
                                                         self.target_callback)
        self._verified_object = None 
        self._timestamp = None

    def target_callback(self, msg): 
        """
        Callback for verified object message. Saves an approximate time of 
        arrival to filter too old messages in the algorithm. 
        Args:
            msg (std_msgs/String): The name of the verified object
        """
        # Save the time of arrival to filter too old messages
        self._timestamp = rospy.Time.now()
        self._verified_object = msg.data
        
    def reset(self): 
        """
        Reset the verified object and timestamp to None 
        """
        self._verified_object = None 
        self._timestamp = None 

    def snap_to_target(self): 
        """
        When speech recognition publishes a verified object from the set of 
        known targets, see if the object is detected in the workspace and snap
        to target

        Returns:
            int, PoseWithCovariance: Target ID and pose 
        """

        target_id = None                 
        target_pose = None 
        
        if self._verified_object is not None and self._timestamp is not None:
            current_time = rospy.Time.now() 

            # The message has to be fresh, < 2s old 
            if (current_time - self._timestamp).to_sec() >= 2: 
                self.reset()
            else:  
                rospy.loginfo("[snap_speech] Verified object received by speech")
                self._detection_filter.update_filtered_detections() 
                target_id = self._verified_object
                pose = self._detection_filter \
                                       .get_target_by_id(self._verified_object)
                if pose is not None: 
                    target_pose = PoseWithCovariance()
                    target_pose.pose = pose 
                    
        return target_id, target_pose


