#!/usr/bin/env python3
from collections import deque
import numpy as np 
import rospy
import threading
from vision_msgs.msg import Detection2DArray 


class TaskDetectionBuffer:

    def __init__(self, detection_capacity=5, buffer_length=100):
        self._object_detection_sub = rospy.Subscriber("/st_cam/objects", 
                                                      Detection2DArray, 
                                                      self.detection_cb)
        # Lock to prevent race condition between detection_cb and get_detection
        self._lock = threading.Lock() 
        self._detection_capacity = detection_capacity
        self._detected_objects = deque([[0.0]*detection_capacity]*buffer_length, 
                                       maxlen=buffer_length)
        
    def detection_cb(self, msg):
        """
        Callback for detected objects. Creates a list of currently detected 
        objects, saves confidence values to detection buffer. 

        Args:
            msg (Detection2DArray): Detection array message 
        """
        with self._lock: 
            objects = [0.0] * self._detection_capacity
            
            if msg.detections:
                for detection in msg.detections:
                    objects[detection.results[0].id] = detection.results[0].score
            self._detected_objects.append(objects)
    
    def is_detected(self, id, threshold=0.65, samples=30): 
        """
        Takes an average over samples of given detection ID, returns True when 
        the detection is successful, false when not. 

        Args:
            id (int): Detection ID 
            threshold (float, optional): The threshold for the required 
                                         confidence score average. Defaults to 
                                         DEFAULT_THRESHOLD.
            samples (int, optional): Defines how many recent samples are 
                                     considered. Defaults to 0.65.

        Returns:
            bool: True, when detection is successful; False when not. 
        """
        
        # safe access 
        with self._lock:
            objects = np.array(self._detected_objects) 
        
        # calculate the mean over the samples 
        confidence_average = np.round(np.mean(objects[-samples,id]),2)
        print(confidence_average)
        if confidence_average > threshold: 
            return True
        return False 
    