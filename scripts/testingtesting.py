#!/usr/bin/python

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty
from threading import Thread

class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])
    def execute(self,userdata):
        rospy.loginfo('initializing')
        rospy.sleep(10)
        if self.preempt_requested:
            self.service_preempt()
            return 'preempted'
        return 'succeeded'

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted','done','notdone'])
    def execute(self,userdata):
        rospy.loginfo('Moving')
        rospy.sleep(10)
        if self.preempt_requested:
            self.service_preempt()
            return 'preempted'
        if userdata.uid==4:
            userdata.uid = 0
            return 'done'
        else:
            userdata.uid+=1
            return 'notdone'
        


def instastopper(outcome_map):
    if outcome_map['Monitor'] == 'invalid':
        return True
    return False

def out_cb(outcome_map):
        return 'done'
    
def out_move_cb(outcome_map):
    if outcome_map['Move'] == 'notdone':
        return 'notdone'
    else:
        return 'done'
    
    

def monitor_cb(ud, msg):
    return False

def create_machine():
    
    init_concurrence = smach.Concurrence(outcomes=['succeeded','preempted'],
                                           default_outcome='',
                                           child_termination_cb=instastopper,
                                           outcome_cb=out_cb)
    with init_concurrence:
        
        smach.Concurrence.add('Initialize',
                               Initialize())

       

        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb))


    static_sm = smach.StateMachine(outcomes=['done'])

    move_concurrence = smach.Concurrence(outcomes=['valid','invalid','preempted'],
                                           default_outcome='valid',
                                           input_keys=['uid'],
                                           output_keys=['uid'],
                                           child_termination_cb=instastopper,
                                           outcome_cb=out_move_cb)
    with move_concurrence:
        smach.Concurrence.add('Move',
                               Move())

       

        smach.Concurrence.add('Monitor', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb))


        
    
    with static_sm:
        #countdown = rospy.get_param('countdown')
        #countdown1 = rospy.Time(10)
        static_sm.userdata.uid = 0 
        smach.StateMachine.add('Idle',
                               smach_ros.MonitorState("/sm_reset", Empty, monitor_cb),
                               transitions={'invalid':'Init',
                                            'valid':'Idle',
                                            'preempted':'Idle'})
        smach.StateMachine.add('Init', init_concurrence, transitions={'valid':'Movement',
                                                                      'invalid':'Idle',
                                                                      'preempted':'Idle'})
        smach.StateMachine.add('Movement', move_concurrence, transitions={'invalid':'Idle',
                                                                          'valid':'Init',
                                                                          'preempted':'Idle'},
                               remapping={'uid':'uid'})

    return static_sm
                               
                                               
        
        


























if __name__ == '__main__':
    rospy.init_node('square_state_machine',log_level=rospy.DEBUG)
    
##    server1 = InitializeServer('init_action')
##    server2 = move_server.MoveAction('square_action')
    
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
