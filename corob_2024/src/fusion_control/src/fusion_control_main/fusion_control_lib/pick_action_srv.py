#!/usr/bin/env python3
#
import string 
import copy 
import rospy 
from fusion_control.msg import PickAction, PickGoal, PickFeedback, PickResult 
from fusion_control_lib import controller_cli
from fusion_control_lib.move_it_action_with_steps import MoveItActionWithSteps,\
                                                         TABLE_LEVEL
from fusion_control_lib.json_utils import read_json_file

JSON_PATH = "/home/alex/user_files/nqnosa/ROBO-666/corob_2024/ros1env/src/json_files/"
TARGETS_PATH = JSON_PATH + "targets.json"
class PickActionNode(MoveItActionWithSteps): 
    """
    Defines the Pick action server
    """
    def __init__(self, action_name: string) -> None: 

        super().__init__(action_name, PickAction, PickGoal(), PickFeedback(),
                         PickResult())
        self.target_data = read_json_file(TARGETS_PATH,"targets")
        controller_cli.set_speed(0.10)  # adjust robot speed here 
        self._goal = None 
        self._pre_grasp_pose = None 
        self._feedback.is_grasping = False 
        self.initialize_steps() 

    def on_preempted(self):
        """
        Stop the robot when pre-empt is requested by the client 
        """
        self.update_feedback("Stop requested.")
        controller_cli.stop() 
        self._server.set_preempted(self._result)

    def initialize_steps(self): 
        """
        Steps that define pick up action 
        """
        self._steps = [self.set_pre_grasp,
                       self.move_to_offset_grasp, 
                       self.open_hand,
                       self.move_to_target,
                       self.grasp_target, 
                       self.move_to_offset_grasp]

    def set_pre_grasp(self): 
        print(self._goal.pose)
        self._pre_grasp_pose = copy.deepcopy(self._goal.pose)
        self._pre_grasp_pose.position.z += 0.15 # m 

    def move_to_offset_grasp(self): 
        """ 
        Move to pre-grasp position 
        """
        if self._pre_grasp_pose.position.z >= TABLE_LEVEL:
            self.update_feedback("Moving to pre-grasp position")
            print(self._pre_grasp_pose)
            controller_cli.plan_waypoints([self._pre_grasp_pose])
            controller_cli.execute_plan()
        else:
            self.update_feedback("Target outside permitted area(under the table)")
        
    def open_hand(self): 
        """
        Open the hand for grasping
        """
        print("Opening hand")
        rospy.sleep(3.0)
        self.update_feedback("Opening hand")
        controller_cli.open_hand() 


    def move_to_target(self): 
        """
        Move to target position

        """
        self._goal.pose.position.z = 0.015
        print(self._goal)
        if self._goal.pose.position.z >= TABLE_LEVEL:
            self.update_feedback("Moving to target")
            print(self._goal.pose)
            controller_cli.plan_waypoints([self._goal.pose])
            controller_cli.execute_plan()
        else:
            self.update_feedback("Target outside permitted area(under the table)")

    def grasp_target(self): 
        """
        Grasp the target 
        """
        target_id = self._goal.target_id
        width = self.get_target_width(target_id)
        force = self.get_target_force(target_id)
        print(width)
        print(force)
        print("Grasping target")
        rospy.sleep(3.0)
        controller_cli.close_hand(width,force) 
        self._feedback.is_grasping = True
        self.update_feedback("Grasped target")

    def move_to_post_grasp(self): 
        """
        Move to post-grasp position 
        """
        self.update_feedback("Moving to post-grasp position")
        controller_cli.plan_waypoints([self._pre_grasp_pose]) 
        controller_cli.execute_plan() 
    
    def get_target_width(self, target_id: int) -> float:
        """
        Gets target width from .json file 
        """
        target_object = self.target_data.get("object",{})
        target = target_object.get(str(target_id), None)

        if target:
            return target.get("required_width", 0.04)
        else:
            return 0.04
        
    def get_target_force(self, target_id: int) -> float:
        """
        Gets target force from .json file 
        """
        target_object = self.target_data.get("object",{})
        target = target_object.get(str(target_id), None)

        if target:
            return target.get("required_force", 5.0)
        else:
            return 5.0
