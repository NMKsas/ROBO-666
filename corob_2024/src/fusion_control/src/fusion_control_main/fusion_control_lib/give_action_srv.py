#!/usr/bin/env python3
#

import string 
import copy 
import rospy
from fusion_control.msg import GiveAction, GiveGoal, GiveFeedback, GiveResult 
from fusion_control_lib import controller_cli
from fusion_control_lib.move_it_action_with_steps import MoveItActionWithSteps,\
                                                         TABLE_LEVEL
from geometry_msgs.msg import PoseStamped

HOME_TF = "panda_link0"
GIVE_POSE = [0.75, -0.2, TABLE_LEVEL + 0.4, 1.0, 0.0, 0.0, 0.0]
HOME_POSE_2 = [0.40, 0.0, TABLE_LEVEL + 0.6, 1.0, 0.0, 0.0, 0.0]
class GiveActionNode(MoveItActionWithSteps): 
    """
    Defines the Give action server
    """
    def __init__(self, action_name: string) -> None: 

        super().__init__(action_name, GiveAction, GiveGoal(), GiveFeedback(),
                         GiveResult())
        
        self._default_home_tf = HOME_TF        
        self._default_home_pose = GIVE_POSE
        self._give_pose = None 
        self._home_pose = None
        self._default_home_pose_2 = HOME_POSE_2

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
        self._steps = [self.set_give_location, 
                       self.move_to_give_position,
                       self.wait,
                       self.open_hand,
                       self.move_to_home_position]

    def set_give_location(self): 
        """
        Initialize give location 
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
        self._give_pose = pose

    def move_to_give_position(self): 
        """
        Move to give position
        """
        
        self.update_feedback("Moving to target")
        controller_cli.plan_waypoints([self._give_pose.pose])
        controller_cli.execute_plan()

    def wait(self): 
        """
        Wait for some seconds
        """
        rospy.sleep(3.)
        
    def open_hand(self): 
        """
        Open the hand for grasping
        """
        self._feedback.is_grasping = False 
        self.update_feedback("Opening hand")
        controller_cli.open_hand() 

    def move_to_home_position(self):

        """
        set and move to home position
        after grasping
        """

        pose2 = PoseStamped() 
        pose2.header.frame_id = self._default_home_tf
        pose2.header.stamp = rospy.Time(0)
        pose2.pose.position.x = self._default_home_pose_2[0]
        pose2.pose.position.y = self._default_home_pose_2[1]
        pose2.pose.position.z = self._default_home_pose_2[2]
        pose2.pose.orientation.x = self._default_home_pose_2[3]
        pose2.pose.orientation.y = self._default_home_pose_2[4]
        pose2.pose.orientation.z = self._default_home_pose_2[5]
        pose2.pose.orientation.w = self._default_home_pose_2[6]
        self._home_pose = pose2
        controller_cli.plan_waypoints([self._home_pose.pose])
        controller_cli.execute_plan()
