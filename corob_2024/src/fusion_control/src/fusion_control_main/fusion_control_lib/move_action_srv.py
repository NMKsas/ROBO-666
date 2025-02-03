#!/usr/bin/env python3
#
#
import string 
import copy 
import rospy 
from fusion_control.msg import MoveAction, MoveGoal, MoveFeedback, MoveResult 
from fusion_control_lib import controller_cli
from fusion_control_lib.move_it_action_with_steps import MoveItActionWithSteps,\
                                                         TABLE_LEVEL

HOME_TF = "panda_link0"
HOME_POSE = [0.4, 0.0, 0.6, 1.0, 0.0, 0.0, 0.0]

class MoveActionNode(MoveItActionWithSteps): 
    """
    Defines the Move action server
    """
    def __init__(self, action_name: string) -> None: 

        super().__init__(action_name, MoveAction, MoveGoal(), MoveFeedback(),
                         MoveResult())
        
        self._goal = None 
        self._current_pose = None

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
        Steps that define the move action 
        """
        self._steps = [self.get_current_pose,
                       self.move]

    def move(self):
        """
        Moves the robot into desired direction or pre defined pose 
        """
        if self._goal.target_id == 3:
            goal_pose = self._current_pose              
            goal_pose.position.y -= 0.1 

            if self._current_pose.position.y < -0.36 or self._current_pose.position.y > 0.36:
                print("movement command outside permitted area")
            else:
                controller_cli.plan_waypoints([self._current_pose])
                controller_cli.execute_plan()


        elif self._goal.target_id == 4:            
            goal_pose = self._current_pose              
            goal_pose.position.y += 0.1 

            if self._current_pose.position.y < -0.36 or self._current_pose.position.y > 0.36:
                print("movement command outside permitted area")
            else:
                controller_cli.plan_waypoints([self._current_pose])
                controller_cli.execute_plan()
        
        
        elif self._goal.target_id == 2:            
            goal_pose = self._current_pose              
            goal_pose.position.x += 0.1 

            if self._current_pose.position.x < 0.33 or self._current_pose.position.x > 0.75:
                print("movement command outside permitted area")
            else:
                controller_cli.plan_waypoints([self._current_pose])
                controller_cli.execute_plan()

        elif self._goal.target_id == 0:
            goal_pose = self._current_pose              
            goal_pose.position.x -= 0.1 

            if self._current_pose.position.x < 0.33 or self._current_pose.position.x > 0.75:
                print("movement command outside permitted area")
            else:
                controller_cli.plan_waypoints([self._current_pose])
                controller_cli.execute_plan()

        elif self._goal.target_id == 1:
            goal_pose = self._current_pose        
            goal_pose.position.z -= 0.1    

            if goal_pose.position.z < TABLE_LEVEL:
                print("movement command outside permitted area")
            else:
                controller_cli.plan_waypoints([goal_pose])
                controller_cli.execute_plan()

        elif self._goal.target_id == 5:
            goal_pose = self._current_pose        
            goal_pose.position.z += 0.1    

            if goal_pose.position.z > (TABLE_LEVEL + 0.8):
                print("movement command outside permitted area")
            else:
                controller_cli.plan_waypoints([goal_pose])
                controller_cli.execute_plan()

        elif self._goal.target_id == 6:
            goal_pose = self._current_pose

            goal_pose.position.x = HOME_POSE[0]
            goal_pose.position.y = HOME_POSE[1]
            goal_pose.position.z = HOME_POSE[2]
            goal_pose.orientation.x = HOME_POSE[3]
            goal_pose.orientation.y = HOME_POSE[4]
            goal_pose.orientation.z = HOME_POSE[5]
            goal_pose.orientation.w = HOME_POSE[6]
            controller_cli.plan_waypoints([goal_pose])
            controller_cli.execute_plan()
 
    def get_current_pose(self):
        self._current_pose = controller_cli.get_pose()
        print(self._current_pose)


