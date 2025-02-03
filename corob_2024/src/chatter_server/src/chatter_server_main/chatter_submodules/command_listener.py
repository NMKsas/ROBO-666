import rospy 
from hri_msgs.msg import LiveSpeech 
from std_msgs.msg import String, Bool, Int16

class CommandListener:
    """
    Command listener to recognize and verify commands. 
    """
    def __init__(self, raw_speech_to_refined_commands,
                 command_list, directions_dict, 
                 locations_dict, objects_dict): 
        self._speech_sub = rospy.Subscriber("/opendr/speech_transcription", 
                                            LiveSpeech, self.speech_command_cb)

        # command lists
        self._raw_speech_to_refined_commands = raw_speech_to_refined_commands
        self._command_list = command_list
        self._direction_dict = directions_dict
        self._location_dict = locations_dict
        self._object_dict = objects_dict

        # relayed, verified topics
        self._stop_pub      = rospy.Publisher("/stop_enabled", 
                                              Bool, queue_size=5)
        self._confirm_pub   = rospy.Publisher("/verification", 
                                              Bool, queue_size=1)
        self._command_pub   = rospy.Publisher("/verified_command", 
                                              String, queue_size=1)
        self._object_pub    = rospy.Publisher("/verified_object", 
                                              Int16, queue_size=1)
        self._location_pub  = rospy.Publisher("/verified_location", 
                                              Int16, queue_size=1)
        self._direction_pub  = rospy.Publisher("/verified_direction", 
                                               Int16, queue_size=1)


    def speech_command_cb(self, ros_transcription_msg):
        """
        Callback when ROS transcription message is received. Verifies valid 
        commands.

        Args:
            ros_transcription_msg (LiveSpeech): Transcripted message from opendr
                                                speech_transcription node. 
        """

        text_raw = ros_transcription_msg.final
        
        # for console debugging
        if text_raw: 
            rospy.loginfo("I heard: " + text_raw)
            # if the raw speech transcription can be interpretted as a valid command
            if text_raw in self._raw_speech_to_refined_commands:
                command = self._raw_speech_to_refined_commands[text_raw]["meaning"]
            else:
                # the raw speech transcription does not correspond to any valid command
                return
        else:
            return
        print(f"command: {command}")

        # Check if the command exists in hard-coded lists. 
        if command == 'stop': 
            self._stop_pub.publish(True)
            rospy.loginfo("STOP enabled")        
        elif command in self._command_list: 
            self._command_pub.publish(command)
            rospy.loginfo("Published command: " + command)
        elif command == 'no':
            self._confirm_pub.publish(False)
            rospy.loginfo("Confirmation: " + command)
        elif command == 'yes':
            self._confirm_pub.publish(True)
            rospy.loginfo("Confirmation: " + command)
        elif command in self._object_dict:
            self._object_pub.publish(self._object_dict[command])
            rospy.loginfo("Published object: " + command)     
        if command in self._location_dict: 
            self._location_pub.publish(self._location_dict[command])
            rospy.loginfo("Published location: " + command)   
        if command in self._direction_dict: 
            self._direction_pub.publish(self._direction_dict[command])
            rospy.loginfo("Published direction: " + command)   
