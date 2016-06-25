import rospy
from actionlib import *
from actionlib_msgs.msg import *
from planner.msg import *
from geometry_msgs.msg import *

class TaskAction(object):
   #setup the move server

    _feedback = taskFeedback()
    _result   = taskResult()
    def __init__(self, name):
        self._action_name = name
        self._as = SimpleActionServer(self._action_name, taskAction,
                                      execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self,goal):
        success = True
        name = goal.name

      #looping over the amount of time
        for s in range(10, 1, -1):
                if self._as.is_preempt_requested():
                #checking for preempts
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
            #random move calculations TODO: calculations that actually make sense
             #de = de-ve.linear.z
             #th = th-ve.angular.z
                self._feedback.info = s
                self._as.publish_feedback(self._feedback)
                rospy.loginfo(self._feedback)
                rospy.Rate(1).sleep

      #what to send after succesfully moving

        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
