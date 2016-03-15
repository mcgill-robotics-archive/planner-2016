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

#comment
## Grabs the goal from the parameter server
def fire_goal_cb(userdata,goal):
    rospy.loginfo("fire_goal_cb")
    rospy.loginfo("random comment")

    counter = userdata.uid

    params = rospy.get_param(uid)
    rospy.loginfo("hasn't crashed yet")

    rospy.loginfo(params)

    time_ = rospy.Time(params['time'][0],params['time'][1])
    velocity = Twist(Vector3(params['velocity'][0],params['velocity'][1],params['velocity'][2]),Vector3(params['velocity'][3],params['velocity'][4],params['velocity'][5]))

    # Torpedo goal
    torpedo_goal = fireGoal(time_,velocity,params['theta'],params['depth'],params['firing'])

    # Store if we need to fire again
    userdata.fire_again = torpedo_goal.fire_again

    userdata.uid += 1

    return torpedo_goal

## Processes whether we have finished firing or not, if we need to fire again,
##  and returns the appropriate outcome.
def fire_result_cb(userdata,status,result):
    rospy.loginfo("fire_result_cb")
    userdata.fire_again = result.fire_again
    if status == GoalStatus.SUCCEEDED:
        if userdata.fire_again==False:
            return 'done'
        else:
            return 'notdone'
    else:
        userdata.fire_again = False
        return 'notdone'

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
    if outcome_map['Monitor']=='invalid' or outcome_map['Torpedo']=='done' or outcome_map['Torpedo']=='notdone':
        return True
    return False

#idle goes to initialize when the button is pressed, hence the seemingly reversed outcomes (invalid means the button was pressed)
def out_idle_cb(outcome_map):
    rospy.loginfo("out_idle_cb")
    rospy.sleep(3.5)
    if outcome_map['Monitor']=='invalid':
        return 'invalid'
    else:
        return 'valid'


# Goes back to idle if it's stopped or once done, and loops if it hasn't firing.
def out_torpedo_cb(outcome_map):
    rospy.loginfo("out_torpedo_cb")
    rospy.sleep(3.5)
    if outcome_map['Monitor'] == 'invalid' or outcome_map['Torpedo']=='done':
        return 'stop'
    elif outcome_map['Torpedo']=='notdone':
        return 'continue'
    else:
        return 'stop'

def create_machine():

    static_sm = smach.StateMachine(outcomes=['done'])

    torpedo_concurrence = smach.Concurrence(outcomes=['done','notdone'],
                                           default_outcome='notdone',
                                           input_keys=['fire_again', 'uid'],
                                           output_keys=['fire_again', 'uid'],
                                           child_termination_cb=instastopper2,
                                           outcome_cb=out_torpedo_cb
                                           )
    with torpedo_concurrence:
        smach.Concurrence.add('Torpedo',
                                    SimpleActionState('torpedo_action',
                                        torpedoAction,
                                        goal_cb=fire_goal_cb,
                                        result_cb=fire_result_cb,
                                        input_keys=['fire_again','uid'],
                                        output_keys=['fire_again','uid'],
                                        outcomes=['done','notdone']
                                    ),
                                    remapping={"torpedo":"torpedo"},
                                )

        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_stop))


################################################################################

    # container for idle. It's a single monitor state, but is in a container to allow delays (to avoid concurrency problems)
    idle_concurrence=smach.Concurrence(outcomes=['valid','invalid'],
                                       default_outcome='valid',
                                       outcome_cb=out_idle_cb)
    with idle_concurrence:
        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Bool, monitor_cb_start))


################################################################################

    #actual state machine, linking the containers together
    with static_sm:
        countdown1 = rospy.Time(10)
        static_sm.userdata.uid = 1
        static_sm.userdata.fire_again = False
        smach.StateMachine.add('Idle',
                               idle_concurrence,
                               transitions={'invalid':'Torpedo',
                                            'valid':'Idle'})
        smach.StateMachine.add('Torpedo', torpedo_concurrence, transitions={'notdone':'Torpedo',
                                                                          'done':'Idle'},
                                                                          remapping={'fire_again':'fire_again', 'uid':'uid'})

        return static_sm


################################################################################





if __name__ == '__main__':

    rospy.init_node('static_state_machine')

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
