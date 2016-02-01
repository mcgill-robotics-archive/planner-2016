#!/user/bin/python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty
from threading import import Thread
from planner import import * 
from planner.msg import * bool
from geometry_msgs.msg import * 

import initalize_server,move_server 


class Fire(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'done', 'notdone'])
    def execute(self, userdata):
        rospy.loginfo('Firing') 
        rospy.sleep(10)
        if self.preempt_requested:
            self.service_preempty()
            return 'preempted' 
        if userdata.uid==1:
            userdata.uid = 0 
            return 'done' 
        else: 
            return 'notdone' 

def instastopper(outcome_map):
    if outcome_map['Monitor'] == 'invalid':
        return True
    return False

def monitor_cb(ud, msg):
    return False

## Grabs the goal from the parameter server, using a counter in userdata to keep track of how many torpedos we have fired

def fire_goal_cb(userdata,goal):

    counter = userdata.uid
    
    params = rospy.get_param('~/torpedo'+str(counter))

    rospy.loginfo(params)

    time_ = rospy.Time(params['time'][0],params['time'][1])

    velocity = Twist(Vector3(params['velocity'][0],params['velocity'][1],params['velocity'][2]),Vector3(params['velocity'][3],params['velocity'][4],params['velocity'][5]))

    torpedo_goal = fireGoal(time_,velocity,params['theta'],params['depth'],params['firing'])

    userdata.uid += 1

    return torpedo_goal

## Processes whether we have finished firing or not and returns the appropriate outcome
def fire_result_cb(userdata,status,goal):
    if status == GoalStatus.SUCCEEDED:
        if userdata.uid==5:
            userdata.uid = 1
            return 'done'
        else:
            # userdata.torpcounter += 1 
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
    rospy.sleep(3.5)
    if outcome_map['Monitor']=='invalid':
        return 'invalid'
    else:
        return 'valid'
#initialize goes to move when it succeeds, goes back to idle if stopped
def out_init_cb(outcome_map):
    rospy.sleep(3.5)
    if outcome_map['Monitor']=='invalid':
        return 'stop'
    elif outcome_map['Initialize']=='succeeded':
        return 'continue'
    else:
        return 'stop'

#move goes back to idle if it's stopped or once done, and loops if it hasn't finished doing all 4 moves   
def out_move_cb(outcome_map):
    rospy.sleep(3.5)
    if outcome_map['Monitor'] == 'invalid' or outcome_map['Torpedo']=='done':
        return 'stop'
    elif outcome_map['Move']=='notdone':
        return 'continue'
    else:
        return 'stop'
    
def create_machine():
	
    static_sm = smach.StateMachine(outcomes=['done'])
    torpedo_concurrence = smach.Concurrence(outcomes=['stop','continue'],
                                           default_outcome='continue',
                                           input_keys=['uid'],
                                           output_keys=['uid'],
                                           child_termination_cb=instastopper2,
                                           outcome_cb=out_move_cb)
    with torpedo_concurrence:
        smach.Concurrence.add('Torpedo',
                                SimpleActionState('torpedo_action',
                                               torpedoAction,
                                               goal_cb=move_goal_cb,
                                               result_cb=move_result_cb,
                                               input_keys=['uid'],
                                               output_keys=['uid'],
                                               outcomes=['done','notdone']))

       
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
                               transitions={'invalid':'Init',
                                            'valid':'Idle'})
        smach.StateMachine.add('Init', init_concurrence, transitions={'continue':'Movement',
                                                                      'stop':'Idle'})
        smach.StateMachine.add('Movement', move_concurrence, transitions={'continue':'Movement',
                                                                          'stop':'Idle'},
                               remapping={'uid':'uid'})

    


    #actual state machine, linking the containers together
    with static_sm:
        #countdown = rospy.get_param('countdown')
        countdown1 = rospy.Time(10)
        static_sm.userdata.uid = 1 
        smach.StateMachine.add('Idle',
                               idle_concurrence,
                               transitions={'invalid':'Init',
                                            'valid':'Idle'})
        smach.StateMachine.add('Init', init_concurrence, transitions={'continue':'Movement',
                                                                      'stop':'Idle'})
        smach.StateMachine.add('Movement', move_concurrence, transitions={'continue':'Movement',
                                                                          'stop':'Idle'},
                               remapping={'uid':'uid'})

    return static_sm    
                
if __name__ == '__main__': 
    rospy.init_node('torp_state_machine',log_level= rospy.DEBUG, sm, '/SM_ROOT')


    server1 = torpedo_server.TorpedoServer('torpedo_action') 



    sm = create_machine()

    sis = smach_ros.IntrospectionServer('tropedo', sm, '/SM_ROOT') 
    sis.start 
    smach_thread = Thread(target=lambda: sm-execute())
    smach_thread.start()

    rospy.on_shutdown(sm.request_preempt)
    sm-execute()
    rospy.spin()

    smach_thread.join()
    sis.stop()
        
            

