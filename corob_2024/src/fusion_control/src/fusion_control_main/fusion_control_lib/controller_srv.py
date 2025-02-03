#!/usr/bin/env python3

# author: Ossi Parikka https://github.com/ozzyuni/LMPVC
# A simple wrapper to launch MoveItController with ROS services enabled

import rospy
from controller import MoveItController
from grasp_client import GraspClient

from fusion_control.srv import ControllerExec, ControllerGetPose,   \
                               ControllerPlan, ControllerSetSpeed,  \
                               ControllerStop
from fusion_control.srv import ControllerExecResponse,      \
                               ControllerGetPoseResponse,   \
                               ControllerPlanResponse,      \
                               ControllerSetSpeedResponse,  \
                               ControllerStopResponse,      \
                               ActivateGripper,             \
                               ActivateGripperResponse


class ControllerServer:
    def __init__(self, controller):
        self.controller = controller
        # Enable all supported services
        self.s1 = rospy.Service('controller_exec', ControllerExec, self.execute)
        self.s2 = rospy.Service('controller_get_pose', ControllerGetPose, self.get_pose)
        self.s3 = rospy.Service('controller_plan', ControllerPlan, self.plan)
        self.s4 = rospy.Service('controller_set_speed', ControllerSetSpeed, self.set_speed)
        self.s5 = rospy.Service('controller_stop', ControllerStop, self.stop)
        self.s6 = rospy.Service('gripper_open', ActivateGripper, self.open_hand)
        self.s7 = rospy.Service('gripper_close', ActivateGripper, self.close_hand)

    def execute(self, req):
        print("Executing!")
        success = self.controller.execute_plan(wait=True)
        print("Execution finished!")
        return ControllerExecResponse(success)
    
    def get_pose(self, req):
        print("Getting pose!")
        pose = self.controller.get_pose()
        return ControllerGetPoseResponse(pose)
    
    def plan(self, req):
        print("Planning!")
        success = self.controller.plan_waypoints(req.waypoints)
        return ControllerPlanResponse(success)
    
    def set_speed(self, req):
        print("Setting speed to", req.speed)
        self.controller.set_speed(req.speed)
        return ControllerSetSpeedResponse(True)
    
    def stop(self, req):
        print("Stopping!")
        self.controller.stop()
        return ControllerStopResponse(True)
    
    def open_hand(self, req):
        print("Opening gripper")
        hand = GraspClient()
        hand.release()
        return ActivateGripperResponse(True)
    
    def close_hand(self, req):
        print("Closing gripper")
        hand = GraspClient()
        hand.grasp()
        return ActivateGripperResponse(True)
    

def controller_srv():

    # Launch a MoveItController instance
    rospy.init_node('lmpvc_controller')
    controller = MoveItController()
    server = ControllerServer(controller=controller)

    # Listen to service calls
    print("Ready to accept commands.")
    rospy.spin()

if __name__ == "__main__":
    controller_srv()