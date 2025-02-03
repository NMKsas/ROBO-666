#!/usr/bin/env python3
import rospy
import threading
from vision_msgs.msg import ObjectHypothesisWithPose

class DetectionSubscriber:

    def __init__(self, threshold=0.0):
        rospy.Subscriber('/opendr/grasp_detected', ObjectHypothesisWithPose, self.detection_cb)
        self._threshold = threshold
        
        # Lock to prevent race condition between detection_cb and get_detection
        self._lock = threading.Lock() 
        self._detections = {}
        self._debug_enabled = False 

    def detection_cb(self, data):
        with self._lock:
            if data.score >= self._threshold:
                pose = data.pose.pose
                self._detections[data.id] = pose
            
            if self._debug_enabled: 
                rospy.loginfo(rospy.get_caller_id() + "Detections updated!")
                for (id, pose) in self._detections.items():
                    print(id)
                    print(pose)
    
    def get_detection(self, id):
        pose = None
        with self._lock:
            pose = self._detections.get(id)
        return pose
    
    def get_detections(self): 
        detections = None 
        with self._lock: 
            detections = self._detections
        return detections
             

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    listener = DetectionSubscriber()
    rospy.loginfo("Detection subscriber node started")
    rospy.spin()
