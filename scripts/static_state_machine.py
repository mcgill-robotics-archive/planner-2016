#!/usr/bin/python

import smach
import smach_ros
import rospy
from threading import Thread
from planner.tasks import *

def create_machine():
    static_sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'])
    
    with static_sm:
        countdown = rospy.getparams('~countdown')
        
        smach.StateMachine.add('Idle',
                               smach_ros.ConditionState(tasks.idle_cond_cb,poll_rate=rospy.Duration(1),max_checks=1),
                               transitions={'true':'Initialize',
                                            'false':'Idle'})
        smach.StateMachine.add('Initialize',
                               SimpleActionState('initaction',
                                               InitializeAction,
                                               goal=countdown),
                               transitions={'succeeded':'Move',
                                            'preempted':'Idle'})
        smach.StateMachine.add('Move',
                               SimpleActionState('squareaction',
                                               MoveAction,
                                               goal=tasks.move_goal_cb,
                                               result=tasks.move_result_cb),
                               transitions={'succeeded':'Move',
                                            'preempted':'Idle',
                                            'done':'Idle'})
                               
                                               
        
        


























if __name__ == '__main__':
    rospy.init_node('square_state_machine')
    sm = create_machine()
    server1 = initialize_server('initaction')
    server2 = move_server('squareaction')
    sis = smach_ros.IntrospectionServer('square_state_machine', sm, '/SM_ROOT')
    sis.start()

    #taken from the threading method used in auv-2015 to avoid problems
    smach_thread = Thread(target=lambda: sm.execute())
    smach_thread.start()

    #taken from auv-2015 to avoid the ctrl+c issue
    rospy.on_shutdown(sm.request_preempt)
    rospy.spin()
    smach_thread.join()
    sis.stop()
    
