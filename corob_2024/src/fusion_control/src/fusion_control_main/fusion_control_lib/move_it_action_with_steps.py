from abc import ABC, abstractmethod
import string 
import rospy 
import actionlib

TABLE_LEVEL = 0.015

class MoveItActionWithSteps(ABC):
    """
    Defines action server for a task with multiple sequential steps 
    """
    def __init__(self, action_name: string, action, goal, feedback, result) -> None: 

        self._action_name = action_name
        self._server = None
        self._steps = None 

        self._action = action
        self._goal = goal
        self._feedback = feedback 
        self._result = result

    @abstractmethod
    def initialize_steps(self):
        """
        Method to define the sequential steps for the action.
        """
        pass
    
    @abstractmethod
    def on_preempted(self): 
        """
        Method to define the actions taken upon pre-empt request
        """
        pass

    def execute_next_step(self, step): 
        """
        Executes next step 

        Args:
            step (function): the next step defined as a function
        """
        step()

    def update_feedback(self, msg): 

        # Log in ros messages
        rospy.loginfo('[%s]: %s', self._action_name, msg)

        # publish feedback to the client 
        self._feedback.feedback_msg = msg
        self._server.publish_feedback(self._feedback)
        
    def action_callback(self, goal) -> None: 
        """
        Action callback. 

        Args:
            goal (ActionGoal): The result sent to client. 
        """
        success = True 

        rospy.loginfo('%s: Executing action' % self._action_name)
        self._goal = goal 

        for step in self._steps: 
            if self._server.is_preempt_requested():
                success = False 
                break                             
            self.execute_next_step(step)
        
        if success: 
            self._result.success = success 
            self.update_feedback("Action succeeded")
            self._server.set_succeeded(self._result)
            
    def start_node(self): 
        """
        Starts the action server with the given action name
        """
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    self._action,
                                                    execute_cb = 
                                                    self.action_callback,
                                                    auto_start=False)
        self._server.register_preempt_callback(self.on_preempted)
        rospy.loginfo(f"{self._action_name} server started")
        self._server.start() 


