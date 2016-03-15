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

import initialize_server,move_server


def octo_goal_cb(userdata,goal):

    counter = userdata.octoid

    params = rospy.get_param(octoid)
    #rospy.loginfo(params)
    time_ = rospy.Time(params['time'][0],params['time'][1])
    velocity = Twist(Vector3(params['velocity'][0],params['velocity'][1],params['velocity'][2]),Vector3(params['velocity'][3],params['velocity'][4],params['velocity'][5]))
    octogon_goal = octogonGoal(time_,velocity,params['theta'],params['depth'],params['claw'])
    #userdata.uid += 1
    return octogon_goal

def octo_result_cb(userdata,status,goal,result):
    if status == GoalStatus.SUCCEEDED:
        userdata.octoid+=1
        if result==True:





def monitor_cb_start(ud, msg):
    if msg.data == True:
        rospy.loginfo('Starting!!')
        return False
    else:
        return True

def monitor_cb_stop(ud, msg):
    if msg.data == False:
        rospy.loginfo('Stopping!')
        return False
    else:
        return True


def instastopper2(outcome_map):
    if outcome_map['Monitor']=='invalid' or outcome_map['Octogon']=='done' or outcome_map['Octogon']=='notdone':
        return True
    return False

def out_octo_cb(outcome_map):
    rospy.sleep(3.5)
    if outcome_map['Monitor'] == 'invalid' or outcome_map['Octogon']=='done':
        return 'stop'
    elif outcome_map['Octogon']=='notdone':
        return 'continue'
    else:
        return 'stop'




def create_machine():
   static_sm = smach.StateMachine(outcomes=['done'])

   octogon_sm = smach.StateMachine(outcomes=['succeeded','failed'])

   with octogon_sm:
       smach.Concurrence.add('Grab',
                                   SimpleActionState('grab_action',
                                                  grabAction,
                                                  goal_cb=grab_goal_cb,
                                                  result_cb=grab_result_cb,
                                                  input_keys=['doubloon'],
                                                  output_keys=['doubloon'],
                                                  outcomes=['done','notdone']),
                                                  transitions={'done' :
                                                  'Surfacing', 'notdone' : 
                                                  'Grab'}
                                                  remapping ={'doubloon' : 
                                                  'doubloon' : 'doubloon'}

       smach.Concurrence.add('Surfacing',
                                   SimpleActionState('surface_action',
                                                  surfaceAction,
                                                  goal_cb=surface_goal_cb,
                                                  result_cb=surface_result_cb,
                                                  input_keys=['retrying'],
                                                  output_keys=['retrying'],
                                                  outcomes=['done','notdone'])
                                                  transitions={'done':'Table',
                                                  'notdone' : 'Surfacing'
                                                  remapping={'retrying': 
                                                  'retrying'})
                                                  

   
	smach.Concurrence.add('Table',
                                   SimpleActionState('surface_action',
                                                  surfaceAction,
                                                  goal_cb=surface_goal_cb,
                                                  result_cb=surface_result_cb,
						  input_keys=['retrying','
						  doubloon'],
                                                  output_keys=['doubloon'
                                                  'retrying'],
                                                  outcomes=['obj1placed',
                                                  'obj2placed',
                                                  'notdone'])
						  transitions={'obj1placed' :
                                                  'grab',obj }, 
						  remapping={'retrying': 
						  'retrying', 'doubloon' : 
						  'doubloon'}






   octogon_concurrence = smach.Concurrence(outcomes=['stop','continue'],
                                           default_outcome='continue',
                                           input_keys=['octoid','doubloon', 'x'],
                                           output_keys=['octoid','doubloon','x'],
                                           child_termination_cb=instastopper2,
                                           outcome_cb=out_move_cb)
     with octogon_concurrence:

    #     smach.Concurrence.add('Octogon',
    #                             SimpleActionState('octogon_action',
    #                                            octogonAction,
    #                                            goal_cb=octo_goal_cb,
    #                                            result_cb=octo_result_cb,
    #                                            input_keys=['octoid','doubloon','x'],
    #                                            output_keys=['octoid','doubloon','x'],
    #                                            outcomes=['done','notdone']))


        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_stop))

     idle_concurrence=smach.Concurrence(outcomes=['valid','invalid'],
                                       default_outcome='valid',
                                       outcome_cb=out_idle_cb)

    with idle_concurrence:
        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_start))



    with static_sm:

        static_sm.userdata.octoid = 0
        smach.StateMachine.add('Idle',
                               idle_concurrence,
                               transitions={'invalid':'Octogon',
                                            'valid':'Idle'})

        smach.StateMachine.add('Octogon', move_concurrence, transitions={'continue':'Octogon',
                                                                          'stop':'Idle'},
                               remapping={'octoid':'octoid',
                                          'doubloon':'doubloon',
                                          'x':'x'})


    return static_sm

























































if __name__ == '__main__':
    rospy.init_node('square_state_machine')

    server1 = octogon_server.OctogonServer('octogon_action')


    sm = create_machine()

    sis = smach_ros.IntrospectionServer('octogon', sm, '/SM_ROOT')
    sis.start  #taken from the threading method used in auv-2015 to avoid problems
    smach_thread = Thread(target=lambda: sm.execute())
    smach_thread.start()

    #taken from auv-2015 to avoid the ctrl+c issue
    rospy.on_shutdown(sm.request_preempt)
    sm.execute()
    rospy.spin()



    smach_thread.join()
    sis.stop()
