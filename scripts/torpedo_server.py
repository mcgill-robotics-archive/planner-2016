import rospy
from actionlib import *
from actionlib_msgs.msg import *
from planner.msg import *
from geometry_msgs.msg import *
import random


class TorpedoAction(object): 

    _feedback = tropFireFeedback()

    _result = tropFireResult() 

    def __init__(self, name): 

        self._action_name = name

        self._as = actionlib,SimpleActionServer(self._action_name, torpedoAction, execute_cb=self._execute_cb, autostart = False) 

        self_as.start() 

    def execute_cb(self,goal):

        #grab all the parameters
      
        success = True
        ti = int(goal.time.secs)
        ve = goal.velocity
        th = goal.theta
        de = goal.depth
        firing = goal.firing 



      #looping over the amount of time
        for s in range(ti,1,-1):

             if self._as.is_preempt_requested():

                #checking for preempts
                rospy.loginfo('%s: Preempted' % self._action_name)

                self._as.set_preempted()

                success = False

                break


            #random move calculations TODO: calculations that actually make sense
             #de = de-ve.linear.z
             #th = th-ve.angular.z

            self._feedback.time_left = rospy.Time().from_sec(ti)

            self._feedback.depth = de

            self._feedback.theta = th

            self._feedback.firing = firing

            self._as.publish_feedback(self._feedback)

            rospy.loginfo(self._feedback)

            rospy.Rate(1).sleep

      #what to send after succesfully moving

        if success:

            # Random logic to state whether we need to fire again or not 
            fireagain = random.randrange(0,2)
            if ( fireagain == 1):
                # fire again
                self._result.fire_again = True
            else: 
                self._result.fire_again = False


            self._result.success = True

            rospy.loginfo('%s: Succeeded' % self._action_name)
            rospy.loginfo('Fire again: %s' % self._result.fire_again) 
            self._as.set_succeeded(self._result)
                
             
          
