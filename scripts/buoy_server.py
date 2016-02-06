import rospy
import actionlib
import actionlib_msgs
import smach
##from actionlib_msgs.msg import *
from planner.msg import *

class ActionServer(object):
    _feedback = planner.msg.buoyFeedback()
    _result = planner.msg.buoyResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, buoyAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self,goal):
         #gets the parameters!

        success = True
        ti = int(goal.time.secs)
        ve = goal.velocity
        th = goal.theta #angle
        de = goal.depth

        for i in range (ti,1,-1):
            if self._as.is_preempt_requested():
                    #checking for preempts
                rospy.loginfo('$s: Preempted' % self.action_name)
                self._as.set_preempted() ##preempted is one of the outcomes
                success = False
                break
                  #calculations go here

                  #======================
            self._feedback.time_ = rospy.Time().from_sec(time)
            self._feedback.depth = de
            self._feedback.theta = th
            self._feedback.velocity = ve
            self._as.pubic_feedback(self._feedback)
            rospy.loginfo(self._feedback)
            rospy.Rate(1).sleep

        if success:
            self._result.success = True
            rospy.loginfo('Buoy Succeeded') ##'%s: Succeeded'% self.buoy_name !!replaced
            self._as.set_succeeded(self._result) ##this is one of the outcomes
