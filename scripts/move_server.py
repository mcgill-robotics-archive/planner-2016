#! /usr/bin/env python

import rospy
import actionlib
import planner.msg

class MoveAction(object):
  
   _feedback = planner.msg.moveFeedback()
   _result   = planner.msg.moveResult()
   def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, planner.msg.moveAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

   def execute_cb(self,msg):
       success = True
       ti = msg.goal.time.to_sec()
       ve = msg.goal.velocity
       th = msg.goal.theta
       de = msg.goal.depth

       for s in range(ti,1.0,-1.0):
             if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
             de = de-ve.linear.z
             th = th-ve.angular.z
             self._feedback.time_left = rospy.Time().from_sec(ti)
             self._feedback.depth = de
             self._feedback.theta = th
             self._as.publish_feedback(self._feedback)
             rospy.Rate(1).sleep

       if success:
           self._result.success = True
           rospy.loginfo('%s: Succeeded' % self._action_name)
           self._as.set_succeeded(self._result)
                
             
            

       

       

