#!/usr/bin/python
import smach
import smach_ros
import rospy
from threading import Thread
from planner import *
from planner.msg import *
from smach_ros import SimpleActionState

from actionlib import *
from actionlib_msgs.msg import *
from std_msgs.msg import Bool
from geometry_msgs.msg import *

import move_server

'''The following callbacks send and receive info from action servers within taskr'''
def move_goal_cb(userdata,goal):
    name_param = rospy.get_param('~/everything/{}/name'.format(userdata.uid))
    return move_goal


def move_result_cb(userdata,status,goal):
    if status == GoalStatus.SUCCEEDED:
            return 'done'
        else:
            return 'notdone'



'''This stops everything (set back to idle) when the kill switch triggers'''
def instastopper2(outcome_map):
    if outcome_map['Monitor'] == 'invalid' or outcome_map['Move']=='done' or outcome_map['Move']=='notdone':
        return True
    return False

'''These 2 callbacks define the behavior of the monitors, what happens when they receive
a particular signal on the topic they are monitoring'''
def monitor_cb_start(ud, msg):
    if msg.data == True:
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


def out_move_cb(outcome_map):
    rospy.sleep(3.5)
    if outcome_map['Monitor'] == 'invalid' or outcome_map['Move'] == 'done':
        return 'stop'
    elif outcome_map['Move'] == 'notdone':
        return 'continue'
    else:
        return 'stop'



task_dict = {}

for tasks in rospy.get_param('~/everything'):
    task_name = str(tasks['name'])
    task_dict­[task_name] = SimpleActionState(­"{}_action".format(task_name),
                                            moveAction,
                                            goal_cb=move_goal_cb,
                                            result_cb=move_result_cb,
                                            input_keys=['uid'],
                                            output_keys=['uid'],
                                            outcomes=['done', 'notdone'])

def create_machine():


    static_sm = smach.StateMachine(outcomes=['succeeded','preempted', 'aborted'])


    static_sm.userdata.uid = 1
    seq = smach.Sequence(
                    outcomes = ['done','notdone']
                    connector_outcome = 'done'
    )

    with seq:
        task_list = rospy.get_param('~/everything')
        for tasks in task_list:
            task_name = str(tasks['name'])
            smach.Sequence.add(task_name,task_dict[task_name])

    execution_concurrence = smach.Concurrence(outcomes=['stop','continue'],
                                          default_outcome='continue',
                                          input_keys=['uid'],
                                          output_keys=['uid'],
                                          child_termination_cb=instastopper2,
                                          outcome_cb=out_move_cb)
    with execution_concurrence:
       smach.Concurrence.add('Move',
                               seq)


       smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_stop))


    init_concurrence = smach.Concurrence(outcomes=['stop','continue'],
                                           default_outcome='continue',
                                           child_termination_cb=instastopper1,
                                           outcome_cb=out_init_cb)
    with init_concurrence:

        paraminit = rospy.get_param('~/everything/initialize')
        countdown1 = rospy.Time(paraminit[0], paraminit[1])

        smach.Concurrence.add('Initialize',
                               SimpleActionState('init_action',
                                               initializeAction,
                                               goal=initializeGoal(countdown=countdown1)))

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

def create_server(task):
    server = taskr.MoveAction('{}_action'.format(task))
    return server

if __name__ == '__main__':
    rospy.init_node('square_state_machine')


    for tasks in rospy.get_param('~/everything'):
        create_server(str(tasks['name']))

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
