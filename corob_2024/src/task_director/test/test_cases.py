#!/usr/bin/env python
from __future__ import print_function
from task_director.srv import * 

import rospy 
import rostest 
import unittest
import sys

PKG = 'task_director'
NAME = 'task_director_test'

ZERO_MASK = int('0000000000000000',2)
FULL_MASK = int('1111111111111111',2)
BITS_IN_MASK = 16 

class TestTaskDirector(unittest.TestCase):
    # After service call is made for /verify_task, task mask is updated 

    def __init__(self, methodName: str = "runTest") -> None:
        super().__init__(methodName)
        rospy.wait_for_service('verify_task')
        self._verify_s = rospy.ServiceProxy('verify_task', VerifyTask)

        rospy.wait_for_service('get_task_progress')
        self._task_progress_s = rospy.ServiceProxy('get_task_progress', TaskProgress)    

        rospy.wait_for_service('reset_task_progress')
        self._reset_s = rospy.ServiceProxy('reset_task_progress', ResetProgress)   

        rospy.wait_for_service('is_fault')
        self._is_fault_s = rospy.ServiceProxy('is_fault', IsFault)   

    def test_verify_task(self):

        # task progress is set 0 initially 
        task_progress_res = self._task_progress_s()
        self.assertEqual(ZERO_MASK, task_progress_res.task_progress)

        # after verification, task progress is set to 1 
        self._verify_s()
        task_progress_res = self._task_progress_s()
        self.assertEqual(int('00000001',2), task_progress_res.task_progress)

    def test_reset_no_fault(self): 
        
        # verify the task to set task progress != 0 
        self._verify_s() 
        task_progress_res = self._task_progress_s()
        self.assertNotEqual(ZERO_MASK, task_progress_res.task_progress)

        # use reset and verify the value is 0 after it 
        self._reset_s()
        task_progress_res = self._task_progress_s()
        self.assertEqual(ZERO_MASK, task_progress_res.task_progress)

        is_fault_res = self._is_fault_s() 
        print(is_fault_res)
        self.assertEqual(False, is_fault_res.is_fault)
        self.assertEqual('', is_fault_res.recovery)
        self.assertEqual('', is_fault_res.fault)


    def test_reset_fault_set(self): 
        """
        To be done or not?  
        """
        pass

    def test_verify_task_overflow(self): 
    
        self._reset_s() 
        for i in range(BITS_IN_MASK): 
            self._verify_s()

        # after 16 tasks are verified, the mask remains 1111 1111 
        task_progress_res = self._task_progress_s() 
        self.assertEqual(FULL_MASK, task_progress_res.task_progress)


if __name__ == '__main__': 
    rostest.rosrun(PKG, NAME, TestTaskDirector, sys.argv)


"""
class CaseA(unittest.TestCase):
    # If amount of tasks is more than the int16 mask allows, user is informed  

    def runTest(self):
        pass

class CaseB(unittest.TestCase):
    # if a fault is detected related to the current task, /is_fault returns 
    # true

    def runTest(self):
        pass

class CaseB(unittest.TestCase):
    # if a fault is detected in previous (critical) task, /is_fault returns 
    # true

    def runTest(self):
        pass

class CaseB(unittest.TestCase):
    # if no fault is detected, returns false 

    def runTest(self):
        pass

"""