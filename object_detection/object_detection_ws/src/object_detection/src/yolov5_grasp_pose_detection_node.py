#!/usr/bin/env python3

"""
This is a node for formatting and publishing target poses for the end effector to grasp objects detected by
yolov5_object_detection_node.

"""
import pyrealsense2 as rs2
import tf2_ros
from scipy.spatial.transform import Rotation
from custom_imports.camera_subscriber import CameraSubscriber
import rospy
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseStamped
from math import pi, atan
import tf2_geometry_msgs
import numpy as np
# from math import isnan
from numpy import isnan

from custom_imports.json_utils import read_json_file
from custom_imports.constants import TARGETS_PATH


NO_ROTATION = 3/2*pi
ROTAION_90_DEGREES = pi

class LocationDataTransformer:
    def __init__(self):
        rospy.init_node('location_data_transformer', anonymous=True)
        self.object_detection_subscriber = rospy.Subscriber("/ee_cam/objects",
                                                            Detection2DArray,
                                                            self.object_detection_callback)
        self.grasping_target_publisher = rospy.Publisher("/opendr/grasp_detected", ObjectHypothesisWithPose, queue_size=1)
        self.camera_subscriber = CameraSubscriber("/ee_cam/color/image_raw",
                                                  "/ee_cam/aligned_depth_to_color/image_raw",
                                                  "/ee_cam/aligned_depth_to_color/camera_info")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.robot_tf_frame = "panda_link0"
        self.camera_tf_frame = "ee_cam_color_optical_frame"

        targets = read_json_file(TARGETS_PATH, "targets")
        self.target_objets = targets["object"]
        

    def get_theta(self, wh_ratio):
        """
        Function for calculating the desired rotation around z-axis for the end effector
        to grasp the target object.

        INPUT PARAMETERS
            :description wh_ratio: Width to height ratio of the bounding box around the target object.
            :type point: Float.

        RETURN VALUES
            :description theta: Needed rotation around the z-axis for the end effector (in radians).
            :type theta: Float.
        """
        if wh_ratio >= 1:
            # if the width of the target object is greater than its height
            theta = ROTAION_90_DEGREES
        else:
            # if the height of the target object is greater than its width
            theta = NO_ROTATION
        
        return theta

    def object_detection_callback(self, detection_msg):
        """
        Callback function for the self.object_detection_subscriber.

        INPUT PARAMETERS
        :descreption detection_msg: Message including the current detection data.
        :type detection_msg: vision_msgs.msg.Detection2DArray.

        RETURN VALUES
        None
        
        """
        if detection_msg.detections:
            for detection in detection_msg.detections:
                result = detection.results[0]
                conf = result.score
                object_id = result.id
                if object_id == 3:
                    return
                object_id_str = str(object_id)
                object_detected = self.target_objets[object_id_str]
                stamp = detection.header.stamp
                bbox = detection.bbox

                # Image corrdinates of the center of the object
                x_img = int(bbox.center.x)
                y_img = int(bbox.center.y)
                
                # find the depth value of the pixel 
                depth_frame = self.camera_subscriber.get_depth()
                # Computing robust depth value as a median of depth values inside l x l square
                # around the center point of the bounding box
                l = 25
                min_y = max(0,y_img-l)
                max_y = min(y_img+l, depth_frame.shape[0]-1)
                min_x = max(0,x_img-l)
                max_x = min(x_img+l, depth_frame.shape[1]-1)
                area_of_interest = depth_frame[min_y:max_y, min_x:max_x]
                valid_depth_values = area_of_interest[area_of_interest != 0]
                d = np.median(valid_depth_values)
                evaluated_depth = d/1000

                intrinsics = self.camera_subscriber.get_intrinsics()
                point3D = rs2.rs2_deproject_pixel_to_point(intrinsics, [x_img, y_img], evaluated_depth)

                for coordinate in point3D:
                    if isnan(coordinate):
                        return

                # Target 3D pose in the frame of the camera attached to the end effector
                target_pose_camera_frame = PoseStamped()
                target_pose_camera_frame.header.frame_id = self.camera_tf_frame
                target_pose_camera_frame.header.stamp = stamp
                
                # Position of the target point
                target_pose_camera_frame.pose.position.x = point3D[0]
                target_pose_camera_frame.pose.position.y = point3D[1]
                target_pose_camera_frame.pose.position.z = point3D[2]

                # Orientation of the target point
                box_width = int(bbox.size_x)
                box_height = int(bbox.size_y)

                if object_id == 1 or object_id == 2:
                    # No rotations needed for all around symmetric objects (the nuts)
                    theta = NO_ROTATION
                else:
                    wh_ratio = box_width/box_height
                    theta = self.get_theta(wh_ratio)

                rot = Rotation.from_euler('xyz', [0, 0, theta], degrees=False)
                quat_rotcmd = rot.as_quat()
                target_pose_camera_frame.pose.orientation.x = quat_rotcmd[0]
                target_pose_camera_frame.pose.orientation.y = quat_rotcmd[1]
                target_pose_camera_frame.pose.orientation.z = quat_rotcmd[2]
                target_pose_camera_frame.pose.orientation.w = quat_rotcmd[3]

                transform = self.tf_buffer.lookup_transform(self.robot_tf_frame, self.camera_tf_frame,
                                                            stamp, rospy.Duration(1.0))
                target_pose_robot_base = tf2_geometry_msgs.do_transform_pose(target_pose_camera_frame, transform)
                target_pose_robot_base.pose.position.z = object_detected['grasping_height']

                msg_out = ObjectHypothesisWithPose()
                msg_out.pose.pose = target_pose_robot_base.pose
                msg_out.id = object_id
                msg_out.score = conf
                self.grasping_target_publisher.publish(msg_out)

    def spin(self):
        rospy.spin()

def main():
    try:
        location_data_transformer = LocationDataTransformer()
        location_data_transformer.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()