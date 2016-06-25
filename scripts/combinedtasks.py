#!/usr/bin/python
import smach
import smach_ros
import rospy
from threading import Thread
from planner import *
from planner.msg import taskGoal, taskAction
from smach_ros import SimpleActionState

from actionlib import *
from actionlib_msgs.msg import *
from std_msgs.msg import Bool
from geometry_msgs.msg import *

import task_server

'''This stops everything (set back to idle) when  the kill switch triggers'''


def instastopper2(outcome_map):
    if outcome_map['Monitor'] == 'invalid' or outcome_map['Movement'] == 'succeeded' or outcome_map['Movement']=='preempted' or outcome_map['Movement'] == 'aborted':
        return True
    return False

'''These 2 callbacks define the behavior of the monitors, what happens when they receive
a particular signal on the topic they are monitoring'''


def monitor_cb_start(ud, msg):
    if msg.data is True:
        rospy.loginfo('Starting!!')
        return False
    else:
        return True


def monitor_cb_stop(ud, msg):
    if msg.data is False:
        rospy.loginfo('Stopping!')
        return False
    else:
        return True

'''What happens when idle and move states finish'''
def out_idle_cb(outcome_map):
    rospy.sleep(3.5)
    if outcome_map['Monitor'] == 'invalid':
        return 'invalid'
    else:
        return 'valid'


def out_task_cb(outcome_map):
    rospy.sleep(3.5)
    return 'succeeded'

task_dict = {}

for tasks in rospy.get_param('~/everything'):
    task_name = tasks[0]
    task_dict[task_name] = SimpleActionState('{}_action'.format(task_name),
                                             taskAction,
                                             goal=taskGoal(name=task_name),
                                             outcomes=['succeeded', 'preempted', 'aborted'])

def create_machine():

    static_sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    static_sm.userdata.uid = 1
    seq = smach.Sequence(
                    outcomes=['succeeded', 'preempted', 'aborted'],
                    connector_outcome='succeeded')

    with seq:
        task_list = rospy.get_param('~/everything')
        for tasks in task_list:
            task_name = tasks[0]
            smach.Sequence.add(task_name,task_dict[task_name])

    execution_concurrence = smach.Concurrence(outcomes=['stop', 'continue'],
                                              default_outcome='continue',
                                              child_termination_cb=instastopper2,
                                              outcome_cb=out_task_cb)
    with execution_concurrence:
        smach.Concurrence.add('Movement',
                              seq)

        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_stop))

    idle_concurrence = smach.Concurrence(outcomes=['valid', 'invalid'],
                                         default_outcome='valid',
                                         outcome_cb=out_idle_cb)

    with idle_concurrence:
        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_start))

    with static_sm:

        smach.StateMachine.add('Idle',
                               idle_concurrence,
                               transitions={'invalid': 'Movement',
                                            'valid': 'Idle'})
        smach.StateMachine.add('Init', init_concurrence, transitions={'continue':'Movement',
                                                                      'stop':'Idle'})
        smach.StateMachine.add('Movement', execution_concurrence, transitions={'succeeded':'Idle',
                                                                               'preempted':'Idle',
                                                                               'aborted':'Idle'})

    return static_sm


if __name__ == '__main__':

    rospy.init_node('square_state_machine')
    rospy.loginfo('hey')
    server = task_server.TaskAction('task_action')

    sm = create_machine()

    sis = smach_ros.IntrospectionServer('square_state_machine', sm, '/SM_ROOT')
    sis.start  #taken from the threading method used in auv-2015 to avoid problems
    smach_thread = Thread(target=lambda: sm.execute())
    smach_thread.start()

    #taken from auv-2015 to avoid the ctrl+c issue
    rospy.on_shutdown(sm.request_preempt)
    sm.execute()
    rospy.spin()

    smach_thread.join()
    sis.stop()
