#!/usr/bin/env python3

EMPTY_MASK = int('0000000000000000',2)
FULL_MASK = int('1111111111111111',2)

class TaskPipeline:
    """
    Class for task pipeline operations 
    """

    def __init__(self):

        self._task_mask =  EMPTY_MASK

    def get_current_task(self): 
        """
        Return the ID of the current task 

        Returns:
            int: task ID 
        """
        return bin(self._task_mask).count('1')


    def get_task_progress(self): 
        """
        Return the current task progress status 

        Returns:
            int: task progress mask 
        """
        return self._task_mask


    def reset_progress(self): 
        """
        Reset the progress by setting mask values zero 
        """
        self._task_mask = EMPTY_MASK

    
    def verify_current_task(self): 
        """
        Mark the current task verified, by bitshifting 1 to the
        task mask. 
        """
        if self._task_mask != FULL_MASK: 
            self._task_mask = (self._task_mask << 1) | 1 
            return True 
        return False 