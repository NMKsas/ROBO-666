from sound_play.libsoundplay import SoundClient
import rospy
from std_msgs.msg import Bool 
from chatter_server.srv import ConfirmTarget, ConfirmTargetResponse, \
                               Confirm, ConfirmResponse, \
                               Feedback, FeedbackResponse, \
                               ListTargets, ListTargetsResponse, \
                               IsObjectDetected, IsObjectDetectedResponse, \
                               ListTools, ListToolsResponse, \
                               TaskInstructions, TaskInstructionsResponse 

from chatter_submodules.detection_filter import DetectionFilter  

OBJECT_TARGET = "object"
LOCATION_TARGET = "location"
DIRECTION_TARGET = "direction"

BITS_IN_MASK = 16 

class FeedbackTalker:
    """
    Offers services for confirming locations and objects. Feedback as speech 
    synthesis. 
    """
    def __init__(self, command_dict, object_dict, direction_dict, location_dict, 
                 task_dict, tool_dict):
        self._detection_filter = DetectionFilter(object_dict)
        self._talker = SoundClient()

        self._command_dict = command_dict
        self._direction_dict = direction_dict
        self._location_dict = location_dict
        self._object_dict = object_dict
        self._task_dict = task_dict
        self._tool_dict = tool_dict
        self._last_msg = "Nothing."

        # Define offered services 
        self._confirm_srv = rospy.Service('confirm', Confirm, 
                                          self.confirm)
        self._confirm_target_srv = rospy.Service('confirm_target', 
                                                  ConfirmTarget, 
                                                  self.confirm_target)
        self._feedback_srv = rospy.Service('feedback', Feedback, 
                                    self.give_feedback)
        self._give_instructions_srv = rospy.Service('give_instructions', 
                                                    TaskInstructions, 
                                                    self.give_instructions)
        self._object_detection_srv = rospy.Service('is_object_detected',
                                                    IsObjectDetected, 
                                                    self.is_object_detected)
        self._list_targets_srv = rospy.Service('list_targets', ListTargets, 
                                                self.list_targets)
        self._list_tools_srv = rospy.Service('list_tools', ListTools, 
                                             self.list_tools)

        
        self._verified_cmd = None 
        self._verified_cmd_sub = rospy.Subscriber('/verification', Bool, 
                                                   self.command_cb)

    def command_cb(self, cmd): 
        """
        Update the heard verification message 

        Args:
            cmd (Bool): True for yes, False for no
        """
        self._verified_cmd = cmd.data

    def utter(self, message): 
        """
        Method to speak the message out loud with speech synthesis. Offers 
        interface to change the speech synthesis method later. 

        Args:
            message (string): message to be spoken out loud
        """
    
        self._talker.say(message)
        rospy.sleep(len(message)*0.075)
        rospy.sleep(1)
        self._last_msg = message
        return
    
    def give_feedback(self, req):
        """
        Utter aloud the received message

        Args:
            req (FeedbackRequest): Contains the message to be read aloud 
        Returns:
            FeedbackResponse: Service response. True when message is uttered.  
        """
        if req.repeat_previous:
            message = self._last_msg
            if not self._last_msg.startswith("I said: "):
                message = "I said: %s" % message
            self.utter(message)
            return FeedbackResponse(True)
        
        if req.message != '': 
            self.utter(req.message)
        return FeedbackResponse(True)
    
    def give_instructions(self, req): 
        """
        ROS1 Service to give instructions on the task at hand.

        Args:
            req (TaskInstructions): ROS1 service request, includes task progress
                                    as integer. Bits are flipped 1 for tasks 
                                    which are already done.  
        Returns:
            TaskInstructionsResponse: ROS1 service response. True after uttering
                                      the instructions. 
        """

        # Iterate through each bit from right to left 
        for i in range(BITS_IN_MASK):
            if not (req.task_progress & (1 << i)): 
                instruction = self._task_dict[i]['instruction']
                if i == 1:
                    self.utter("You have finished %d task. Next you need to %s" % 
                            (i, instruction))
                else:
                    self.utter("You have finished %d tasks. Next you need to %s" % 
                            (i, instruction))
                
                break 
        return TaskInstructionsResponse(True)


    def is_object_detected(self, req): 
        """
        Find out whether any object is recognized in the workspace
        Args:
            req (IsObjectDetectedRequest): Service request for defining 
                                           condition; is object detected
        Returns:
            IsObjectDetectedResponse: Service response. True if an object is 
                                      seen, False for none seen. 
        """
        
        self._detection_filter.update_filtered_detections()
        target_len = len(self._detection_filter.get_filtered_detections())
        if target_len != 0: 
            return IsObjectDetectedResponse(True)
        self.utter(f"No object seen.")
        return IsObjectDetectedResponse(False)
    
    def list_targets(self, req): 
        """
        List targets (objects/locations) aloud 
        Args:
            req (ListTargetsRequest): Service request. Contains target type 
                                      (object/location)
        """
        target_type = req.target_type
        if target_type == OBJECT_TARGET:
            return self.list_objects(req.action)
        if target_type == LOCATION_TARGET: 
            return self.list_locations()
        if target_type == DIRECTION_TARGET: 
            return self.list_directions()
        
    def list_directions(self): 
        """
        List all the known directions aloud 

        Returns:
            ListTargetsResponse: Service response. True when directions have 
                                 been listed aloud. 
        """

        direction_list = []
        directions_added = 0
        num_of_directions = len(self._direction_dict.values())
        for direction in self._direction_dict.values():
            direction_list.append(direction)
            directions_added += 1
            if directions_added < num_of_directions and num_of_directions > 1:
                direction_list.append("! ")
        if num_of_directions > 1:
            direction_list.insert(-1, "and ")

        concat_list = ''.join(direction_list)
        self.utter(f"I currently know directions: {concat_list}. \
                     Which direction should be used?")
        return ListTargetsResponse(True) 

    def list_locations(self): 
        """
        List all the known locations aloud 

        Returns:
            ListTargetsResponse: Service response. True when locations have 
                                 been listed aloud. 
        """
        location_list = []
        locations_added = 0
        num_of_locations = len(self._location_dict.values())
        for location in self._location_dict.values():
            location_list.append(location)
            locations_added += 1
            if locations_added < num_of_locations and num_of_locations > 1:
                location_list.append("! ")
        if num_of_locations > 1:
            location_list.insert(-1, "and ")

        concat_list = ''.join(location_list)
        self.utter(f"I currently know locations: {concat_list}. \
                     Which location should be used?")
        return ListTargetsResponse(True)

    def list_objects(self, action): 
        """
        List seen objects aloud. 

        Args:
            action_name (int): ID number of the action. 
                               Used for correct conjugation.
        Returns:
            ListTargetsResponse: Service response. True when the list or objects
                                 is uttered. 
 
        """
        for attempt in range(5):
            # update detections and get target names 
            self._detection_filter.update_filtered_detections()
            detections_dict = self._detection_filter.get_filtered_detections()

            if len(detections_dict) == 0: 
                self.utter(f"No object seen.")
                return ListTargetsResponse(True)
            else: 
                target_names = []
                num_of_objects = len(detections_dict)
                objects_added = 0
                for id in detections_dict.keys():
                    target_names.append(self._object_dict[id])
                    objects_added += 1
                    if objects_added < num_of_objects and num_of_objects > 1:
                        target_names.append(", ")
                if num_of_objects > 1:
                    target_names.insert(-1, "and ")
                concat_list = ''.join(target_names)
                self.utter(f"I see: {concat_list}. Which item should be {self._command_dict[action]['past_participle']}?")
                return ListTargetsResponse(True) 
        rospy.loginfo("No object seen. Error?")
        return ListTargetsResponse(False)

    def list_tools(self, req):
        """
        Lists the needed tools. The request message includes tool IDs which 
        are read from the tools.json.

        Args:
            req (ListTools): ROS1 service request

        Returns:
            ListToolsResponse: True, after the tools are listed. 
        """

        tools_msg = ["You need:"]

        # Iterate through each bit from right to left 
        tools_no = 0 
        for i in range(BITS_IN_MASK):
            if req.tool_mask & (1 << i): 
                tool = self._tool_dict[i]
                msg = tool['name']
                size = tool['size']
                unit = tool['unit']
                if size != "": 
                    msg += " of size %s %s," % (size, unit)
                tools_msg.append(msg)
                tools_no += 1
        
        if tools_no > 1: 
            tools_msg.insert(-1, "and")
        tools_msg.append("to complete the task.")
        msg = ", ".join(tools_msg)
        self.utter(msg)
        return ListToolsResponse(True)


    def confirm_target(self, req): 
        """
        Service call for confirming target (location/object). 

        Args:
            req (ConfirmTargetRequest): ROS1 service request. 
                                        See details in srv/.

        Returns:
            ConfirmTargetResponse:  Service response.  
                                    True if a confirmation is received,
                                    False otherwise. 
        """
        target_id = req.target_id
        target_type = req.target_type
        
        if target_type == 'object': 
            target_name = self._object_dict[target_id]
        elif target_type == 'location': 
            target_name = self._location_dict[target_id]
        elif target_type == 'direction': 
            target_name = self._direction_dict[target_id]
        else: 
            return ConfirmTargetResponse(False)

        # Define feedback messages
        feedback_on_yes = f"Verified {target_type}: {target_name}"
        feedback_on_no = f"Discarded {target_type}."
        feedback_on_none = f"Discarded {target_type}."

        # TODO: The next line could be a feedback service in the node tree? 
        self.utter(f"Snapped to {target_type} {target_name}. Confirm {target_type}?")
        
        # poll for confirmation 
        success = self.confirm_with_fb(feedback_on_yes, feedback_on_no,
                                       feedback_on_none)

        return ConfirmTargetResponse(success)
    
    def confirm_start(self,req): 
        """
        Poll confirmation for starting the assembly pipeline. 

        Args:
            req (Confirm): Confirm service request 
        """
        # Define feedback messages
        feedback_on_yes = f"Assembly process started."
        feedback_on_no = f""
        feedback_on_none = f"Canceling task process."

        success = self.confirm_with_fb(feedback_on_yes, feedback_on_no,
                                       feedback_on_none, req.num_attempts)
        return ConfirmResponse(success)
    
    def confirm(self, req): 
        """
        Verify user confirms the action with "yes" or "no" speech input. 

        Args:
            req (ConfirmRequest): ROS1 Service request for confirmation
        """

        self._verified_cmd = None 
        
        for attempt in range(req.num_attempts): 
            if self._verified_cmd is True: 
                return ConfirmResponse(True)
            elif self._verified_cmd is False: 
                return ConfirmResponse(False)
            else:
                rospy.loginfo(f"attempt no. {attempt}")
                rospy.sleep(1)

        self.utter("No confirmation heard.")
        return ConfirmResponse(False) 
    
    
    def confirm_with_fb(self, feedback_on_yes, feedback_on_no, feedback_on_none, 
                num_attempts=10): 
        """
        Verify that the action is confirmed. 

        Args:
            feedback_on_yes (str): Feedback given when yes is received
            feedbacl_on_no (str): Feedback given when no is received  

        Returns:
            bool: True if verified, False if declined 
        """

        # initialize command empty before polling 
        self._verified_cmd = None 

        for attempt in range(num_attempts):
            if self._verified_cmd is True: 
                self.utter(feedback_on_yes)
                return True
            elif self._verified_cmd is False:
                self.utter(feedback_on_no)
                return False
            else: 
                rospy.loginfo(f"attempt no. {attempt}")
                rospy.sleep(1)

        self.utter("No confirmation heard. " + feedback_on_none)
        return False 