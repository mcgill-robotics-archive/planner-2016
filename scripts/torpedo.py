#!/usr/bin/python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from threading import Thread
from planner import *
from planner.msg import *
from geometry_msgs.msg import *
from actionlib import *
from actionlib_msgs.msg import *
import initialize_server,move_server,torpedo_server
from smach_ros import SimpleActionState


## Grabs the goal from the parameter server
def fire_goal_cb(userdata,goal):


    counter = userdata.uid

    params = rospy.get_param('~/torpedo'+str(counter))
    rospy.loginfo(params)
    rospy.loginfo("WHAT")
    time_ = rospy.Time(params['time'][0],params['time'][1])
    velocity = Twist(Vector3(params['velocity'][0],params['velocity'][1],params['velocity'][2]),Vector3(params['velocity'][3],params['velocity'][4],params['velocity'][5]))

    # Torpedo goal
    torpedo_goal = fireGoal(time_,velocity,params['theta'],params['depth'],params['firing'])

    # Store if we need to fire again
    #userdata.fire_again = torpedo_goal.fire_again

    #userdata.uid += 1

    return torpedo_goal

## Processes whether we have finished firing or not, if we need to fire again,
##  and returns the appropriate outcome.
def fire_result_cb(userdata,status,result):
    rospy.loginfo("result")
<<<<<<< HEAD
    result.fire_again = userdata.fire_again
    if status == GoalStatus.SUCCEEDED:
        if result.fire_again==False:
            # What does uid do?
            #userdata.uid = 1
            return 'done'
        else:
            userdata.fire_again = False
            return 'notdone'
=======
    if status == GoalStatus.SUCCEEDED:
        #if result.fire_again==False:
            # What does uid do?
            #userdata.uid = 1
        return 'done'
        #else:
        #    userdata.fire_again = False
        #    return 'notdone'
>>>>>>> c330dd3dffa6dab10e685bff705b7f56a8abd6fb

#gets called when the button sends a signal
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
    rospy.loginfo("Insta2")
    if outcome_map['Monitor']=='invalid' or outcome_map['Torpedo']=='done' or outcome_map['Torpedo']=='notdone':
        return True
    return False

#idle goes to initialize when the button is pressed, hence the seemingly reversed outcomes (invalid means the button was pressed)
def out_idle_cb(outcome_map):
    rospy.sleep(3.5)
    rospy.loginfo("out_idle_cb")
    if outcome_map['Monitor']=='invalid':
        return 'invalid'
    else:
        return 'valid'


# Goes back to idle if it's stopped or once done, and loops if it hasn't firing.
def out_torpedo_cb(outcome_map):
    rospy.sleep(3.5)
    rospy.loginfo("out_torpedo_cb")
<<<<<<< HEAD

=======
>>>>>>> c330dd3dffa6dab10e685bff705b7f56a8abd6fb
    if outcome_map['Monitor'] == 'invalid' or outcome_map['Torpedo']=='done':
        return 'stop'
    elif outcome_map['Torpedo']=='notdone':
        return 'continue'
    else:
        return 'stop'

def create_machine():

    static_sm = smach.StateMachine(outcomes=['done'])

    #container for idle. It's a single monitor state, but is in a container to allow delays (to avoid concurrency problems)
    idle_concurrence=smach.Concurrence(outcomes=['valid','invalid'],
                                       default_outcome='valid',
                                       outcome_cb=out_idle_cb)

    with idle_concurrence:
        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_start))


################################################################################

    torpedo_concurrence = smach.Concurrence(outcomes=['stop','continue'],
                                           default_outcome='continue',
                                           input_keys=['uid','fire_again'],
                                           output_keys=['uid','fire_again'],
                                           child_termination_cb=instastopper2,
<<<<<<< HEAD
                                           outcome_cb=out_torpedo_cb
=======
                                           #outcome_cb=out_torpedo_cb
>>>>>>> c330dd3dffa6dab10e685bff705b7f56a8abd6fb
                                           )
    with torpedo_concurrence:
        smach.Concurrence.add('Torpedo',
                                SimpleActionState('torpedo_action',
                                               torpedoAction,
<<<<<<< HEAD
                                               goal_cb=fire_goal_cb,
                                               result_cb=fire_result_cb,
=======
                                               #goal_cb=fire_goal_cb,
                                               #result_cb=fire_result_cb,
>>>>>>> c330dd3dffa6dab10e685bff705b7f56a8abd6fb
                                               input_keys=['uid','fire_again'],
                                               output_keys=['uid','fire_again'],
                                               outcomes=['done','notdone', ]))


        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_stop))

################################################################################

    #actual state machine, linking the containers together
    with static_sm:
        #countdown = rospy.get_param('countdown')
        countdown1 = rospy.Time(10)
        static_sm.userdata.uid = 1
        static_sm.userdata.fire_again = False
        smach.StateMachine.add('Idle',
                               idle_concurrence,
                               transitions={'invalid':'Torpedo',
                                            'valid':'Idle'})
        #smach.StateMachine.add('Init', init_concurrence, transitions={'continue':'Movement',
        #                                                              'stop':'Idle'})
        smach.StateMachine.add('Torpedo', torpedo_concurrence, transitions={'continue':'Torpedo',
                                                                          'stop':'Idle'},
                               remapping={'uid':'uid', 'fire_again':'fire_again'})

    return static_sm
################################################################################





if __name__ == '__main__':
    rospy.init_node('torpedo_state_machine')

    server1 = torpedo_server.TorpedoServer('torpedo_action')



    sm = create_machine()

    sis = smach_ros.IntrospectionServer('torpedo', sm, '/SM_ROOT')
    sis.start
    smach_thread = Thread(target=lambda: sm.execute())
    smach_thread.start()

    rospy.on_shutdown(sm.request_preempt)
    sm.execute()
    rospy.spin()

    smach_thread.join()
    sis.stop()
