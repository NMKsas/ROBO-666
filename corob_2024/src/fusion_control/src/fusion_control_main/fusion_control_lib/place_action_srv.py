#!/usr/bin/env python3
#
import string 
import copy 
import rospy
from fusion_control.msg import PlaceAction, PlaceGoal, PlaceFeedback, \
                               PlaceResult 
from fusion_control_lib import controller_cli
from fusion_control_lib.move_it_action_with_steps import MoveItActionWithSteps,\
                                                         TABLE_LEVEL
from geometry_msgs.msg import PoseStamped

HOME_TF = "panda_link0" 
HOME_POSE = [0.60, 0.0, TABLE_LEVEL + 0.4, 1.0, 0.0, 0.0, 0.0]
DEFAULT_PLACE = [0.50, 0.2, TABLE_LEVEL, 1.0, 0.0, 0.0, 0.0]

class PlaceActionNode(MoveItActionWithSteps): 
    """
    Defines the Place action server
    """
    def __init__(self, action_name: string) -> None: 

        super().__init__(action_name, PlaceAction, PlaceGoal(), PlaceFeedback(),
                         PlaceResult())

        self._default_home_pose = HOME_POSE
        self._default_home_tf = HOME_TF

        self._goal = None 
        self._pre_grasp_pose = None 
        self._post_grasp_pose = None
        self._feedback.is_grasping = True
        self.initialize_steps() 

    def on_preempted(self):
        """
        Stop the robot when pre-empt is requested by the client 
        """
        self.update_feedback("Stop requested.")
        controller_cli.stop() 

    def initialize_steps(self): 
        """
        Steps that define pick up action 
        """
        self._steps = [self.set_place_target,
                       self.set_pre_grasp,
                       self.set_post_grasp,
                       self.move_to_pre_grasp, 
                       self.move_to_target,
                       self.open_hand,
                       self.move_to_post_grasp]

    def set_place_target(self): 
        """ 
        If no goal is given, use default place 
        """
        pose = PoseStamped() 
        pose.header.frame_id = self._default_home_tf
        pose.header.stamp = rospy.Time(0)
        pose.pose.position.x = DEFAULT_PLACE[0]
        pose.pose.position.y = DEFAULT_PLACE[1]
        pose.pose.position.z = DEFAULT_PLACE[2]
        pose.pose.orientation.x = DEFAULT_PLACE[3]
        pose.pose.orientation.y = DEFAULT_PLACE[4]
        pose.pose.orientation.z = DEFAULT_PLACE[5]
        pose.pose.orientation.w = DEFAULT_PLACE[6]
        self._goal = pose

    def set_pre_grasp(self): 
        """
        Initialize pre grasp
        """
        self._pre_grasp_pose = copy.deepcopy(self._goal.pose)
        self._pre_grasp_pose.position.z = TABLE_LEVEL + 0.15 # m 

    def set_post_grasp(self): 
        """
        Initialize post grasp 
        """
        pose = PoseStamped() 
        pose.header.frame_id = self._default_home_tf
        pose.header.stamp = rospy.Time(0)
        pose.pose.position.x = self._default_home_pose[0]
        pose.pose.position.y = self._default_home_pose[1]
        pose.pose.position.z = self._default_home_pose[2]
        pose.pose.orientation.x = self._default_home_pose[3]
        pose.pose.orientation.y = self._default_home_pose[4]
        pose.pose.orientation.z = self._default_home_pose[5]
        pose.pose.orientation.w = self._default_home_pose[6]
        self._post_grasp_pose = pose

    def move_to_pre_grasp(self): 
        """ 
        Move to defined location 
        """
        print(self._pre_grasp_pose)
        self.update_feedback("Moving to pre-grasp position")
        controller_cli.plan_waypoints([self._pre_grasp_pose])
        controller_cli.execute_plan()

    def move_to_target(self): 
        """
        Move to target position
        """
        
        self.update_feedback("Moving to target")
        self._goal.pose.position.z = TABLE_LEVEL + 0.05 # m 
        controller_cli.plan_waypoints([self._goal.pose])
        controller_cli.execute_plan()
        
    def open_hand(self): 
        """
        Open the hand for grasping
        """
        self.update_feedback("Opening hand")
        rospy.sleep(3.0)
        controller_cli.open_hand() 
        self._feedback.is_grasping = False 
        self.update_feedback("Hand opened")

    def move_to_post_grasp(self): 
        """
        Move to post-grasp position 
        """
        self.update_feedback("Moving to post-grasp position")
        controller_cli.plan_waypoints([self._post_grasp_pose.pose]) 
        controller_cli.execute_plan() 