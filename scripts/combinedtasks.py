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
    counter = userdata.uid
    check = rospy.get_param('~/everything/treasure')
    rospy.loginfo(check)
    time_params = rospy.get_param('~/everything/pathmarkers/time')
    vel_params = rospy.get_param('~/everything/pathmarkers/velocity')
    pose = vel_params['Pose']
    twist = vel_params['Twist']
    rospy.loginfo(time_params)
    rospy.loginfo(vel_params)
    time_ = rospy.Time(time_params['seconds'],time_params['milliseconds'])
    velocity = Twist(Vector3(pose['x'],pose['y'],pose['z']),
                     Vector3(twist['x'],twist['y'],twist['z']))
    move_goal = moveGoal(time_,velocity,vel_params['Theta'],vel_params['Depth'])
    userdata.uid += 1
    return move_goal


def move_result_cb(userdata,status,goal):
    if status == GoalStatus.SUCCEEDED:
        if userdata.uid==5:
            userdata.uid = 1
            return 'done'
        else:
            return 'notdone'





# def bin_goal_cb(userdata,goal):
#     counter = userdata.uid
#
#     params = rospy.get_param('~/everything)
#     rospy.loginfo(params)
#     time_ = rospy.Time(params['time'][0],params['time'][1])
#     velocity = Twist(Vector3(params['velocity'][0],params['velocity'][1],params['velocity'][2]),Vector3(params['velocity'][3],params['velocity'][4],params['velocity'][5]))
#     move_goal = moveGoal(time_,velocity,params['theta'],params['depth'])
#     userdata.uid += 1
#     return move_goal
#
# ## Processes whether we have done 4 moves or not and returns the appropriate outcome
# def bin_result_cb(userdata,status,goal):
#     if status == GoalStatus.SUCCEEDED:
#         if userdata.uid==5:
#             userdata.uid = 1
#             return 'done'
#         else:
#             return 'notdone'
#
#
#
#
#
#
#
# def fire_goal_cb(userdata,goal):
#
#     counter = userdata.uid
#
#     params = rospy.get_param()
#     rospy.loginfo("hasn't crashed yet")
#
#     rospy.loginfo(params)
#
#     time_ = rospy.Time(params['time'][0],params['time'][1])
#     velocity = Twist(Vector3(params['velocity'][0],params['velocity'][1],params['velocity'][2]),Vector3(params['velocity'][3],params['velocity'][4],params['velocity'][5]))
#
#     # Torpedo goal
#     torpedo_goal = fireGoal(time_,velocity,params['theta'],params['depth'],params['firing'])
#
#     # Store if we need to fire again
#     userdata.fire_again = torpedo_goal.fire_again
#
#     userdata.uid += 1
#
#     return torpedo_goal
#
# ## Processes whether we have finished firing or not, if we need to fire again,
# ##  and returns the appropriate outcome.
# def fire_result_cb(userdata,status,result):
#     rospy.loginfo("fire_result_cb")
#     userdata.fire_again = result.fire_again
#     if status == GoalStatus.SUCCEEDED:
#         if userdata.fire_again==False:
#             return 'done'
#         else:
#             return 'notdone'
#     else:
#         userdata.fire_again = False
#         return 'notdone'
#
# def octo_goal_cb(userdata,goal):
#
#     counter = userdata.octoid
#
#     params = rospy.get_param()
#     #rospy.loginfo(params)
#     time_ = rospy.Time(params['time'][0],params['time'][1])
#     velocity = Twist(Vector3(params['velocity'][0],params['velocity'][1],params['velocity'][2]),Vector3(params['velocity'][3],params['velocity'][4],params['velocity'][5]))
#     octogon_goal = octogonGoal(time_,velocity,params['theta'],params['depth'],params['claw'])
#     #userdata.uid += 1
#     return octogon_goal
#
# def octo_result_cb(userdata,status,goal,result):
#     if status == GoalStatus.SUCCEEDED:
#         userdata.octoid+=1
#         if result==True:
#
#
#

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

#
# def instastopperocto(outcome_map):
#     if outcome_map['Monitor']=='invalid' or outcome_map['Octogon']=='done' or outcome_map['Octogon']=='notdone':
#         return True
#     return False





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
#
# def out_bin_cb(outcome_map):
#     rospy.sleep(3.5)
#     if outcome_map['Monitor'] == 'invalid' or outcome_map['Bins']=='done':
#         return 'stop'
#     elif outcome_map['Bins']=='notdone':
#         return 'continue'
#     else:
#         return 'stop'
#
# def out_torpedo_cb(outcome_map):
#     rospy.loginfo("out_torpedo_cb")
#     rospy.sleep(3.5)
#     if outcome_map['Monitor'] == 'invalid' or outcome_map['Torpedo']=='done':
#         return 'stop'
#     elif outcome_map['Torpedo']=='notdone':
#         return 'continue'
#     else:
#         return 'stop'
#
#
# def out_octo_cb(outcome_map):
#     rospy.sleep(3.5)
#     if outcome_map['Monitor'] == 'invalid' or outcome_map['Octogon']=='done':
#         return 'stop'
#     elif outcome_map['Octogon']=='notdone':
#         return 'continue'
#     else:
#         return 'stop'


def create_machine():

    # params = rospy.get_param('/everything')

    static_sm = smach.StateMachine(outcomes=['done'])

    #octogon_sm = smach.StateMachine(outcomes=['succeeded','failed'])

    static_sm.userdata.uid = 1
    move_concurrence = smach.Concurrence(outcomes=['stop','continue'],
                                          default_outcome='continue',
                                          input_keys=['uid'],
                                          output_keys=['uid'],
                                          child_termination_cb=instastopper2,
                                          outcome_cb=out_move_cb)
    with move_concurrence:
       smach.Concurrence.add('Move',
                               SimpleActionState('square_action',
                                              moveAction,
                                              goal_cb=move_goal_cb,
                                              result_cb=move_result_cb,
                                              input_keys=['uid'],
                                              output_keys=['uid'],
                                              outcomes=['done','notdone']))


       smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_stop))



    #  bin_concurrence = smach.Concurrence(outcomes=['stop','continue'],
    #                                      default_outcome='continue',
    #                                      input_keys=['uid'],
    #                                      output_keys=['uid'],
    #                                      child_termination_cb=instastopper2,
    #                                      outcome_cb=out_move_cb)
    #  with bin_concurrence:
    #      smach.Concurrence.add('Move',
    #                           SimpleActionState('square_action',
    #                                          moveAction,
    #                                          goal_cb=move_goal_cb,
    #                                          result_cb=move_result_cb,
    #                                          input_keys=['uid'],
    #                                          output_keys=['uid'],
    #                                          outcomes=['done','notdone']))
    #
    #
    #   smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_stop))
    #
    #
    #
    #
    # with octogon_sm:
    #     smach.Concurrence.add('Grab',
    #                                 SimpleActionState('grab_action',
    #                                                grabAction,
    #                                                goal_cb=grab_goal_cb,
    #                                                result_cb=grab_result_cb,
    #                                                input_keys=['doubloon'],
    #                                                output_keys=['doubloon'],
    #                                                outcomes=['done','notdone']),
    #                                                transitions={'done' :
    #                                                'Surfacing', 'notdone' :
    #                                                'Grab'}
    #                                                remapping ={'doubloon' :
    #                                                'doubloon' : 'doubloon'}
    #
    #     smach.Concurrence.add('Surfacing',
    #                                 SimpleActionState('surface_action',
    #                                                surfaceAction,
    #                                                goal_cb=surface_goal_cb,
    #                                                result_cb=surface_result_cb,
    #                                                input_keys=['retrying'],
    #                                                output_keys=['retrying'],
    #                                                outcomes=['done','notdone'])
    #                                                transitions={'done':'Table',
    #                                                'notdone' : 'Surfacing'
    #                                                remapping={'retrying':
    #                                                'retrying'})
    #
    #
    #
    # smach.Concurrence.add('Table',
    #                                 SimpleActionState('table_action',
    #                                                tableAction,
    #                                                goal_cb=table_goal_cb,
    #                                                result_cb=table_result_cb,
    # 			  input_keys=['retrying','doubloon'],
    #                                                output_keys=['doubloon'
    #                                                'retrying'],
    #                                                outcomes=['obj1placed',
    #                                                'obj2placed',
    #                                                'notdone'])
    # 			  transitions={'obj1placed' :
    #                                                'grab','obj' },
    # 			  remapping={'retrying':
    # 			  'retrying', 'doubloon' :
    # 			  'doubloon'}
    #
    #
    #
    #
    #
    #
    # octogon_concurrence = smach.Concurrence(outcomes=['stop','continue'],
    #                                         default_outcome='continue',
    #                                         input_keys=['octoid','doubloon', 'x'],
    #                                         output_keys=['octoid','doubloon','x'],
    #                                         child_termination_cb=instastopper2,
    #                                         outcome_cb=out_move_cb)
    #   with octogon_concurrence:
    #
    #       smach.Concurrence.add('Octogon',
    #                               SimpleActionState('octogon_action',
    #                                              octogonAction,
    #                                              goal_cb=octo_goal_cb,
    #                                              result_cb=octo_result_cb,
    #                                              input_keys=['octoid','doubloon','x'],
    #                                              output_keys=['octoid','doubloon','x'],
    #                                              outcomes=['done','notdone']))
    #
    #
    #      smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_stop))
    #
    idle_concurrence = smach.Concurrence(outcomes=['valid', 'invalid'],
                                         default_outcome='valid',
                                         outcome_cb=out_idle_cb)

    with idle_concurrence:
        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_start))

    with static_sm:

        pathmarkers = rospy.get_param('~/everything/pathmarkers')
        smach.StateMachine.add('Idle',
                               idle_concurrence,
                               transitions={'invalid': 'Movement',
                                            'valid': 'Idle'})
        for keys in pathmarkers:
            smach.StateMachine.add('Movement'+str(pathmarkers['keys']['name']), move_concurrence, transitions={'continue': 'Movement',
                                                                          'stop': 'Idle'},
                                                                          remapping={'uid': 'uid'})

        # for keys in params.anchorbins:
        #
        #     smach.StateMachine.add(keys.name, bins_concurrence, transitions={'continue':keys.name,
        #                                                                   'stop':'Idle'},
        #                                                                   remapping={'uid':'uid'})
        # for keys in params.torpedoes:
        #
        #     smach.StateMachine.add(keys.name, torpedo_concurrence, transitions={'notdone':keys.name,
        #                                                                   'done':'Idle'},
        #                                                                   remapping={'fire_again':'fire_again', 'uid':'uid'})
        #
        #
        # for keys in params.treasure:
        #
        #     smach.StateMachine.add('Octogon', octogon_concurrence, transitions={'continue':'Octogon',
        #                                                                   'stop':'Idle'},
        #                                                                   remapping={'octoid':'octoid',
        #                                                                   'doubloon':'doubloon',
        #                                                                   'x':'x'})

    return static_sm


if __name__ == '__main__':
    rospy.init_node('square_state_machine')

    #server1 = octogon_server.OctogonServer('octogon_action')
    #server1 = initialize_server.InitializeServer('init_action')
    server2 = move_server.MoveAction('square_action')

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
