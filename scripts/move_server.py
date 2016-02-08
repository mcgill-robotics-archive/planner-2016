<<<<<<< HEAD


import rospy
from actionlib import *
from actionlib_msgs.msg import *
from planner.msg import *
from geometry_msgs.msg import *

class MoveAction(object):
   #setup the move server
=======
#! /usr/bin/env python

import rospy
from actionlib import *
from planner.msg import *

class MoveAction(object):
>>>>>>> create torpedo task
  
   _feedback = moveFeedback()
   _result   = moveResult()
   def __init__(self, name):
    self._action_name = name
<<<<<<< HEAD
    self._as = SimpleActionServer(self._action_name, moveAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

   def execute_cb(self,goal):
      #grab all the parameters
      
       success = True
       ti = int(goal.time.secs)
       ve = goal.velocity
       th = goal.theta
       de = goal.depth

      #looping over the amount of time
       for s in range(ti,1,-1):
             if self._as.is_preempt_requested():
                #checking for preempts
=======
    self._as = actionlib.SimpleActionServer(self._action_name, moveAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

   def execute_cb(self,msg):
       success = True
       ti = msg.goal.time.to_sec()
       ve = msg.goal.velocity
       th = msg.goal.theta
       de = msg.goal.depth

       for s in range(ti,1.0,-1.0):
             if self._as.is_preempt_requested():
>>>>>>> create torpedo task
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
<<<<<<< HEAD
            #random move calculations TODO: calculations that actually make sense
             #de = de-ve.linear.z
             #th = th-ve.angular.z
=======
             de = de-ve.linear.z
             th = th-ve.angular.z
>>>>>>> create torpedo task
             self._feedback.time_left = rospy.Time().from_sec(ti)
             self._feedback.depth = de
             self._feedback.theta = th
             self._as.publish_feedback(self._feedback)
<<<<<<< HEAD
             rospy.loginfo(self._feedback)
             rospy.Rate(1).sleep

      #what to send after succesfully moving

=======
             rospy.Rate(1).sleep

>>>>>>> create torpedo task
       if success:
           self._result.success = True
           rospy.loginfo('%s: Succeeded' % self._action_name)
           self._as.set_succeeded(self._result)
                
             
            

       

       

