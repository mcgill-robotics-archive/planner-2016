import rospy
from actionlib import *
from actionlib_msgs.msg import *
from planner.msg import *
from geometry_msgs.msg import *

class TaskAction(object):
   #setup the move server

    _feedback = taskFeedback()
    _result = taskResult()

    def __init__(self, name):
        self._action_name = name
        self._as = SimpleActionServer(self._action_name, taskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        success = True
        name = goal.name
        #rospy.Rate(5).sleep()


      #what to send after succesfully moving

        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            rospy.Rate(5).sleep()
            self._as.set_succeeded(self._result)
