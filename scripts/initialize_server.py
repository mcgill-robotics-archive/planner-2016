
<<<<<<< HEAD
from geometry_msgs.msg import *
=======

>>>>>>> create torpedo task
import rospy
from actionlib import *
from actionlib_msgs.msg import *
from planner.msg import *
import random

class InitializeServer(object):
    _feedback = initializeFeedback()
    _result = initializeResult()
<<<<<<< HEAD

=======
    
>>>>>>> create torpedo task


    def __init__(self, name):
        
        
<<<<<<< HEAD
        self._as = SimpleActionServer(name, initializeAction, execute_cb=self.execute_cb, auto_start=False)
        
        
        self._as.start()
=======
        self._as = SimpleActionServer(name, initializeAction, execute_cb=self.execute_cb, auto_start=True)
        
        
        #self._as.start()
>>>>>>> create torpedo task

    def execute_cb(self,goal):
        rospy.loginfo('starting')
        t = goal
<<<<<<< HEAD
        rospy.loginfo(goal)
        seconds = int(goal.countdown.secs) #floating point
        nanoseconds = goal.countdown.nsecs

        for s in range(seconds,0,-1):
             if self._as.is_preempt_requested():
                rospy.loginfo('Initialize Preempted')
=======
        seconds = t.to_sec() #floating point
        nanoseconds = t.to_nsec()

        for s in range(t,1.0,-1.0):
             if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
>>>>>>> create torpedo task
                self._as.set_preempted()
                success = False
                break
             #rand = random.randint(0,1)
            
                
             self._feedback.time_left = rospy.Time.from_sec(s)
             self._as.publish_feedback(self._feedback)
<<<<<<< HEAD
             rospy.loginfo(self._feedback)
             #if rand == 0:
             
             #break
             rospy.Rate(1).sleep()

        self._result.offset = Pose(Point(10,10,10),Quaternion(20,20,20,20))
        rospy.loginfo('Initialize Succeeded')
        self._as.set_succeeded(self._result)




##        rospy.loginfo('Initialize Preempted')
##        self._as.set_preempted() 
            
=======
             #if rand == 0:
             self._result.offset = geometry_msgs.Pose(Point(10,10,10),Quaternion(20,20,20,20))
             rospy.loginfo('%s: Succeeded' % self._action_name)
             self._as.set_succeeded(self._result)
             break
             rospy.Rate(1).sleep()


        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted() 
        
>>>>>>> create torpedo task

            
        
