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

##gets the goals for non-static stuff
def buoy_goal_cb(userdata,goal):
    counter = userdata.uid
##This is where to get parameters
    params = rospy.get_param( ##change this stuff to param name in yaml## '~/move'+str(counter))
##    #rospy.loginfo(params)
    time_ = rospy.Time(params['time'][0],params['time'][1])
    velocity = Twist(Vector3(params['velocity'][0],params['velocity'][1],params['velocity'][2]),Vector3(params['velocity'][3],params['velocity'][4],params['velocity'][5]))
    buoy_goal = buoyGoal(time_,velocity,params['theta'],params['depth'],velocity)
    userdata.uid += 1
    return buoy_goal
#
##receives goals and do stuff
def buoy_result_cb(userdata,status,goal):
    if status == GoalStatus.SUCCEEDED:
        counter +=1
        return 'done'
    else:
        return 'notdone'

def instastopper1(outcome_map):
    if outcome_map['Monitor'] == 'invalid' or outcome_map['buoy']=='succeeded' or outcome_map['buoy']=='not done':
        return True ##This terminates the container
    return False

#gets called when the button (abort) sends a signal
def monitor_cb_start(ud, msg):
    if msg.data == True:
        rospy.loginfo('Starting')
        return False
    else:
        return True

def monitor_cb_stop(ud, msg):
    if msg.data == False:
        rospy.loginfo('Stopping')
        return False
    else:
        return True

def out_buoy_cb(outcome_map):
    rospy.sleep(3.5) ##pauses for 3.5 lengths of time
    if outcome_map['Monitor'] == 'invalid' or outcome_map['buoy']=='done':
        return 'stop'
    elif outcome_map['buoy']=='notdone':
        return 'continue'
    else:
        return 'stop'

def create_machine():
    #containers have at least a monitor state that checks the button, plus the actual action state running (except Idle)

    #container for initialize
    static_sm = smach.StateMachine(outcomes=['done'])

    buoy_concurrence = smach.Concurrence(outcomes=['stop','continue'],
                                           default_outcome='continue',
                                           child_termination_cb=instastopper1, ##instastopper thing
                                           outcome_cb=out_buoy_cb)
    with buoy_concurrence:

        ##paraminit = rospy.get_param('~/initialize')
        ##countdown1 = rospy.Time(paraminit[0], paraminit[1])

        smach.Concurrence.add('buoy',
                               SimpleActionState('buoy_action',
                                               buoy,
                                               goal=buoyGoal(time = rospy.time, velocity = Twist(Vector3(params['velocity'][0],params['velocity'][1],params['velocity'][2]),Vector3(params['velocity'][3],params['velocity'][4],params['velocity'][5])))

        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_stop))

    #container for idle. It's a single monitor state, but is in a container to allow delays (to avoid concurrency problems)
    idle_concurrence=smach.Concurrence(outcomes=['valid','invalid'],
                                       default_outcome='valid',
                                       outcome_cb=out_idle_cb)

    with idle_concurrence:
        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_start))

#actual state machine, linking the containers together
    with static_sm:
        #countdown = rospy.get_param('countdown')
        countdown1 = rospy.Time(10)
        static_sm.userdata.uid = 1
        smach.StateMachine.add('Idle',
                               idle_concurrence,
                               transitions={'invalid':'buoy',
                                            'valid':'Idle'})
        smach.StateMachine.add('buoy', buoy_concurrence, transitions={'continue':'Movement',
                                                                      'stop':'Idle'})
        smach.StateMachine.add('Movement', move_concurrence, transitions={'continue':'Movement',
                                                                          'stop':'Idle'},
                               remapping={'uid':'uid'})


    return static_sm




if __name__ == '__main__':
    rospy.init_node('square_state_machine')

    server1 = buoy_server.BuoyServer('buoy_action')

    sm = create_machine()

    sis = smach_ros.IntrospectionServer('buoy_state_machine', sm, '/SM_ROOT')
    sis.start  #taken from the threading method used in auv-2015 to avoid problems
    smach_thread = Thread(target=lambda: sm.execute())
    smach_thread.start()

    #taken from auv-2015 to avoid the ctrl+c issue
    rospy.on_shutdown(sm.request_preempt)
    sm.execute()
    rospy.spin()



    smach_thread.join()
    sis.stop()
