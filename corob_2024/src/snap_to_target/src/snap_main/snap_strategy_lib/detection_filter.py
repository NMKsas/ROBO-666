#!/usr/bin/env python3
from snap_strategy_lib.detector_sub import DetectionSubscriber

WORKSPACE_LIMIT_X = [0.33, 0.75]
WORKSPACE_LIMIT_Y = [-0.36, 0.36]
ACCEPTED_OBJECTS_DICT = {'0':'extension bar', '1':'nut 10 mm',
                            '2': 'nut 13 mm', '3':'rachet',
                            '4':'socket 10', '5': 'socket 13'}

class DetectionFilter:
    """ 
    Helper class to filter wanted detections out of other rubbish 
    """

    def __init__(self, accepted_objects_dict=ACCEPTED_OBJECTS_DICT,
                 workspace_limits=[WORKSPACE_LIMIT_X, WORKSPACE_LIMIT_Y]):
        
        self._detections = DetectionSubscriber()
        self._filtered_detections = {}
        self._workspace_limits = workspace_limits 
        self._accepted_objects_dict=accepted_objects_dict

    def update_filtered_detections(self): 
        """
        Filter the detections based on workspace limits and 
        accepted categories, update filtered lists when called 
        """
        detections = self._detections.get_detections()
        
        # clear old values 
        self._filtered_detections.clear()

        # add only targets from accepted categories, within the workspace limits
        for (id, pose) in detections.items():
            if id in self._accepted_objects_dict:
                # update lists 
                self._filtered_detections[id] = pose
            
    def get_target_by_id(self, id): 
        """
        Return target by ID
        """
        if id not in self._filtered_detections: 
            return None 
        return self._filtered_detections[id]

    def get_filtered_detections(self):
        """
        Return filtered detections  
        """ 
        return self._filtered_detections        
