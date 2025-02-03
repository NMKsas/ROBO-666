import message_filters as mf
import threading 
import rospy

from sensor_msgs.msg import Image as ROS_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs2

class CameraSubscriber: 

    def __init__(self,
                 image_topic,
                 depth_topic,
                 depth_camera_info_topic): 
        
        # initialize topic subscribers 
        self._camera_info_sub = rospy.Subscriber(depth_camera_info_topic, 
                                                 CameraInfo, 
                                                 self.camera_info_cb,
                                                 queue_size=1)
        self._rgb_sub = mf.Subscriber(image_topic, ROS_Image)
        self._depth_sub = mf.Subscriber(depth_topic, ROS_Image)

        # synchronize depth and rgb streams
        self._ts = mf.ApproximateTimeSynchronizer([self._rgb_sub, 
                                                   self._depth_sub], 
                                                   queue_size=5, slop=20.0, 
                                                   allow_headerless=True)
        self._ts.registerCallback(self.images_cb)

        self._rgb_in = None 
        self._depth_in = None 
        self._camera_intrinsics = None 

        self._bridge = CvBridge() 

        self._lock_rgb = threading.Lock() 
        self._lock_depth = threading.Lock()
        self._lock_intrinsics = threading.Lock() 

    def camera_info_cb(self, msg):
        """
        Get camera info from the aligned layer
        """
        if self._depth_in is not None:
            self._camera_intrinsics = rs2.intrinsics()
            self._camera_intrinsics.width = msg.width
            self._camera_intrinsics.height = msg.height
            self._camera_intrinsics.ppx = msg.K[2]
            self._camera_intrinsics.ppy = msg.K[5]
            self._camera_intrinsics.fx = msg.K[0]
            self._camera_intrinsics.fy = msg.K[4]
            if msg.distortion_model == 'plumb_bob':
                self._camera_intrinsics.model = rs2.distortion \
                                                   .modified_brown_conrady
            self._camera_intrinsics.coeffs = [i for i in msg.D]

            # camera information fetched, unregister the subscriber
            self._camera_info_sub.unregister()

    def images_cb(self, image_data, depth_data): 
        """
        Callback for the regular image. The images are saved in cv2 format. 
        """
        with self._lock_rgb: 
            self._rgb_in = self._bridge.imgmsg_to_cv2(image_data, 
                                                      image_data.encoding)
        with self._lock_depth:
            desired_encoding = depth_data.encoding
            depth_data.encoding = desired_encoding
            self._depth_in = self._bridge.imgmsg_to_cv2(depth_data, desired_encoding=desired_encoding)

    def get_depth(self):
        with self._lock_depth:
            return self._depth_in
    
    def get_rgb(self): 
        with self._lock_rgb:
            return self._rgb_in

    def get_intrinsics(self):
        with self._lock_intrinsics: 
            return self._camera_intrinsics 