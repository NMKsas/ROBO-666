#!/usr/bin/env python3
# Copyright 2020-2024 OpenDR European Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This is a slightly edited version of object_detection_2d_yolov5_node.py developed by OpenDR. The reason for the minor changes I have made is
that I wanted to use the weights of my own YOLOv5-model trained with custom data. Also, I needed to add a header with time stamp to the output
image message. All the changes I have made are notated like this:

###   My changes < ###
...
(the actual code I've changed)
...
### > My changes   ###

The original version of this node can be found from OpenDR's GitHub page: https://github.com/opendr-eu/opendr.

"""
import argparse
import torch
from time import perf_counter

import rospy
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image as ROS_Image
from opendr_bridge import ROSBridge

from opendr.engine.data import Image


###   My changes < ###
# Using my own version of YOLOv5DetectorLearner and my own model weights
from custom_imports.yolov5_learner import YOLOv5DetectorLearner
from custom_imports.constants import EE_OBJECT_DETECTION_WEIGHTS
from custom_imports.constants import ST_OBJECT_DETECTION_WEIGHTS

### > My changes   ###

from opendr.perception.object_detection_2d import draw_bounding_boxes




class ObjectDetectionYOLOV5Node:

    def __init__(self, input_rgb_image_topic="/usb_cam/image_raw",
                 output_rgb_image_topic="/opendr/image_objects_annotated", detections_topic="/opendr/objects",
                 performance_topic=None, device="cuda", model_name="yolov5s"):
        """
        Creates a ROS Node for object detection with YOLOV5.
        :param input_rgb_image_topic: Topic from which we are reading the input image
        :type input_rgb_image_topic: str
        :param output_rgb_image_topic: Topic to which we are publishing the annotated image (if None, no annotated
        image is published)
        :type output_rgb_image_topic: str
        :param detections_topic: Topic to which we are publishing the annotations (if None, no object detection message
        is published)
        :type detections_topic:  str
        :param performance_topic: Topic to which we are publishing performance information (if None, no performance
        message is published)
        :type performance_topic:  str
        :param device: device on which we are running inference ('cpu' or 'cuda')
        :type device: str
        :param model_name: network architecture name
        :type model_name: str
        """
        self.input_rgb_image_topic = input_rgb_image_topic

        if output_rgb_image_topic is not None:
            self.image_publisher = rospy.Publisher(output_rgb_image_topic, ROS_Image, queue_size=1)
        else:
            self.image_publisher = None

        if detections_topic is not None:
            self.object_publisher = rospy.Publisher(detections_topic, Detection2DArray, queue_size=1)
        else:
            self.object_publisher = None

        if performance_topic is not None:
            self.performance_publisher = rospy.Publisher(performance_topic, Float32, queue_size=1)
        else:
            self.performance_publisher = None

        self.bridge = ROSBridge()

        # Initialize the object detector

        ###   My changes < ###
        # Enabeling the use of my own custom model
        weights = None
        if input_rgb_image_topic == "/ee_cam/color/image_raw":
            weights = EE_OBJECT_DETECTION_WEIGHTS
        elif input_rgb_image_topic == "/st_cam/color/image_raw":
            weights = ST_OBJECT_DETECTION_WEIGHTS

        if weights is not None:
            self.object_detector = YOLOv5DetectorLearner(model_name=model_name, path=weights, device=device)
        else:
            self.object_detector = YOLOv5DetectorLearner(model_name=model_name, device=device)
        ### > My changes   ###



    def listen(self):
        """
        Start the node and begin processing input data.
        """
        rospy.init_node('opendr_object_detection_yolov5_node', anonymous=True)
        rospy.Subscriber(self.input_rgb_image_topic, ROS_Image, self.callback, queue_size=1, buff_size=10000000)
        rospy.loginfo("Object detection YOLOV5 node started.")
        rospy.spin()

    def callback(self, data):
        """
        Callback that processes the input data and publishes to the corresponding topics.
        :param data: input message
        :type data: sensor_msgs.msg.Image
        """
        if self.performance_publisher:
            start_time = perf_counter()
        # Convert sensor_msgs.msg.Image into OpenDR Image
        
        ###   My changes < ###
        # Saving the original header since it needs to be included into the outgoing message in order to make mp4 files
        # from rosbags including the outgoing messages
        original_header = data.header
        ### > My changes   ###
        image = self.bridge.from_ros_image(data, encoding='bgr8')

        # Run object detection
        boxes = self.object_detector.infer(image)

        if self.performance_publisher:
            end_time = perf_counter()
            fps = 1.0 / (end_time - start_time)  # NOQA
            fps_msg = Float32()
            fps_msg.data = fps
            self.performance_publisher.publish(fps_msg)

        # Publish detections in ROS message
        if self.object_publisher is not None:
            self.object_publisher.publish(self.bridge.to_ros_bounding_box_list(boxes))

        if self.image_publisher is not None:
            # Get an OpenCV image back
            image = image.opencv()
            # Annotate image with object detection boxes
            image = draw_bounding_boxes(image, boxes, class_names=self.object_detector.classes, line_thickness=3)
            # Convert the annotated OpenDR image to ROS2 image message using bridge and publish it

            ###   My changes < ###
            # Including the header to the message
            img_msg_out = self.bridge.to_ros_image(Image(image), encoding='bgr8')
            img_msg_out.header = original_header
            self.image_publisher.publish(img_msg_out)
            ### > My changes   ###
            


def main():
    parser = argparse.ArgumentParser()
    ###   My changes < ###
    # The default value is changed by me
    # parser.add_argument("-i", "--input_rgb_image_topic", help="Topic name for input rgb image",
    #                     type=str, default="/ee_cam/color/image_raw")
    parser.add_argument("-i", "--input_rgb_image_topic", help="Topic name for input rgb image",
                        type=str, default="/camera/color/image_raw")
    ### > My changes   ###
    parser.add_argument("-o", "--output_rgb_image_topic", help="Topic name for output annotated rgb image",
                        type=lambda value: value if value.lower() != "none" else None,
                        default="/opendr/image_objects_annotated")
    parser.add_argument("-d", "--detections_topic", help="Topic name for detection messages",
                        type=lambda value: value if value.lower() != "none" else None,
                        default="/opendr/objects")
    parser.add_argument("--performance_topic", help="Topic name for performance messages, disabled (None) by default",
                        type=str, default=None)
    parser.add_argument("--device", help="Device to use, either \"cpu\" or \"cuda\", defaults to \"cuda\"",
                        type=str, default="cuda", choices=["cuda", "cpu"])
    parser.add_argument("--model_name", help="Network architecture, defaults to \"yolov5s\"",
                        type=str, default="yolov5s", choices=['yolov5s', 'yolov5n', 'yolov5m', 'yolov5l', 'yolov5x',
                                                              'yolov5n6', 'yolov5s6', 'yolov5m6', 'yolov5l6', 'custom'])
    args = parser.parse_args(rospy.myargv()[1:])

    try:
        if args.device == "cuda" and torch.cuda.is_available():
            device = "cuda"
        elif args.device == "cuda":
            print("GPU not found. Using CPU instead.")
            device = "cpu"
        else:
            print("Using CPU.")
            device = "cpu"
    except:
        print("Using CPU.")
        device = "cpu"

    object_detection_yolov5_node = ObjectDetectionYOLOV5Node(device=device, model_name=args.model_name,
                                                             input_rgb_image_topic=args.input_rgb_image_topic,
                                                             output_rgb_image_topic=args.output_rgb_image_topic,
                                                             detections_topic=args.detections_topic,
                                                             performance_topic=args.performance_topic)
    object_detection_yolov5_node.listen()


if __name__ == '__main__':
    main()
