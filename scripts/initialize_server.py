#! /usr/bin/env python

import rospy
import actionlib
import planner.msg
import random

class InitializeAction(object):
    _feedback = planner.msg.initializeFeedback()
    _result = planner.msg.initializeResult()



    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, planner.msg.initializeAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self,msg):
        
        t = msg.goal
        seconds = t.to_sec() #floating point
        nanoseconds = t.to_nsec()

        for s in range(t,1.0,-1.0):
             if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
             rand = random.randint(0,1)
            
                
             self._feedback.time_left = rospy.Time.from_sec(s)
             self._as.publish_feedback(self._feedback)
             if rand == 0:
                 self._result.offset = Pose(Point(10,10,10),Quaternion(20,20,20,20))
                 rospy.loginfo('%s: Succeeded' % self._action_name)
                 self._as.set_succeeded(self._result)
                 break
             rospy.Rate(1).sleep()


        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
                
                
 
        

            
        
