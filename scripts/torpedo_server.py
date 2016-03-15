import rospy
from actionlib import *
from actionlib_msgs.msg import *
from planner.msg import *
from geometry_msgs.msg import *
import random

#comment
class TorpedoServer(object):

    _feedback = torpedoFeedback()

    _result = torpedoResult()

    def __init__(self, name):

        rospy.loginfo("server started")
        self._action_name = name

        self._as = SimpleActionServer(self._action_name, torpedoAction, execute_cb=self.execute_cb, auto_start = False)

        self._as.start()

    def execute_cb(self,goal):
        rospy.loginfo("Server execute_cb")

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

             self._feedback.time_left = rospy.Time().from_sec(ti)

             self._feedback.depth = de

             self._feedback.theta = th

             self._feedback.firing = firing

             self._as.publish_feedback(self._feedback)

             rospy.loginfo(self._feedback)

             rospy.Rate(1).sleep

      # What to send after succesfully moving

        if success:

            rospy.loginfo("Server recieved sucess on fire action")
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

# Rasins
