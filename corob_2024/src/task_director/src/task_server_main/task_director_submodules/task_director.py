import rospy 
from task_director.srv import Inspection, InspectionResponse,       \
                              IsFault, IsFaultResponse,             \
                              ResetProgress, ResetProgressResponse, \
                              TaskProgress, TaskProgressResponse,   \
                              VerifyTask, VerifyTaskResponse,       \
                              GetTools, GetToolsResponse

from task_director_submodules.json_utils import read_json_file
from task_director_submodules.task_detector_buffer import TaskDetectionBuffer
from task_director_submodules.task_pipeline import TaskPipeline

EMPTY_MASK = int('0000000000000000',2)

class TaskDirector:
    """
    Task progress server. Provides ROS1 services for handling tasks.    
    """
    def __init__(self, detections_file_path, fault_file_path, task_file_path): 
        
        self._detection_buffer = TaskDetectionBuffer()
        self._task_pipeline = TaskPipeline() 


        self._get_tools_srv = rospy.Service('get_tools', GetTools,
                                             self.get_tools)
        self._inspect_task_srv = rospy.Service('inspect_task', Inspection,
                                                self.inspect_task)
        self._is_fault_srv = rospy.Service('is_fault', IsFault, 
                                            self.check_faults)
        self._reset_progress_srv = rospy.Service('reset_task_progress', 
                                                  ResetProgress,
                                                  self.reset_srv)
        self._task_progress_srv = rospy.Service('get_task_progress', 
                                                 TaskProgress, 
                                                 self.task_progress_srv)
        self._verify_task_srv = rospy.Service('verify_task', VerifyTask,
                                               self.verify_task_srv)
        
        # TODO: JSON reads happen outside the class? 
        self._detect_list = read_json_file(detections_file_path, 'detections')
        self._fault_list = read_json_file(fault_file_path, 'fault_detections')
        self._task_list = read_json_file(task_file_path, 'tasks')

        rospy.loginfo("Task director started")        


    def check_faults(self, request): 
        """
        Check that critical fault detections are not detected for already 
        performed tasks.  
        Args:
            request (IsFault): ROS1 service request
        """
    
        latest_task = self._task_pipeline.get_current_task()

        # iterate over completed tasks and the current task 
        #for task_id in range(latest_task + 1): 
        task_id = latest_task 
        fault_indices = self._task_list[task_id]['fault_detections']
        for fault_id in fault_indices: 

            # if any of the fault IDs is detected, fault exists 
            if self._detection_buffer.is_detected(fault_id): 
                fault = self._detect_list[fault_id]['fault']
                recovery = self._detect_list[fault_id]['recovery']
                return IsFaultResponse(True, fault, recovery)
            
        return IsFaultResponse(False,"","")

    def get_tools(self, request):
        """
        ROS1 service to get tools for the current task.

        Args:
            request (GetTools): ROS1 service call. 
        Returns:
            GetToolsResponse: int16 mask for the existing tools  
        """

        current_task_id = self._task_pipeline.get_current_task() 
        tools = EMPTY_MASK
        # flip bits for each tool index 
        for tool_id in self._task_list[current_task_id]['tools']: 
            tools |= (1 << tool_id)

        return GetToolsResponse(tools)

    def inspect_task(self, request):
        """
        ROS1 service to inspect the current task. Task id is tied to 
        a specific object id. 

        Args:
            request (Inspection): ROS1 service request to visually verify task
                                  is done, i.e., the task id is detected with 
                                  high enough confidence.  

        Returns:
            InspectionResponse: True, when the
        """
        
        recovery = ""
        success = False
        current_task_id = self._task_pipeline.get_current_task() 
        required_detections = self._task_list[current_task_id] \
                                             ['required_detections']
        
        # no detections required, only operator verification needed
        if len(required_detections) == 0:
            feedback = "I cannot visually verify the task."
            success = True
            return InspectionResponse(feedback, recovery, success)

        # check all the required detections are seen 
        for id in required_detections: 
            if not self._detection_buffer.is_detected(id):             
                feedback = self._detect_list[id]['fault']
                recovery = self._detect_list[id]['recovery']
                return InspectionResponse(feedback, recovery, success) 
        
        feedback = "Visual inspection succeeded."
        success = True
        return InspectionResponse(feedback, recovery, success)

    
    def reset_srv(self, request): 
        """
        ROS1 service to reset the current task progress

        Args:
            request (ResetProgress): ROS1 service request

        Returns:
            ResetProgressResponse: True when reset is done (successful) 
        """
        self._task_pipeline.reset_progress() 
        return ResetProgressResponse(True)
    
    def task_progress_srv(self, request):
        """
        ROS1 service to fetch the current task progress

        Args:
            request (TaskProgress): ROS1 service request

        Returns:
            TaskProgressResponse: int16 mask of the current task pipeline 
        """
        return TaskProgressResponse(self._task_pipeline.get_task_progress()) 
    
    def verify_task_srv(self, request): 
        """
        ROS1 service to verify the current task. 

        Args:
            request (VerifyTask): ROS1 service request

        Returns:
            VerifyTaskResponse: True, when task is successfully verified. False,
                                when the task pipeline has reached its end and 
                                no verification can be done
        """
        return VerifyTaskResponse(self._task_pipeline.verify_current_task())
