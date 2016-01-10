#!/usr/bin/python

import smach
import smach_ros
import rospy
from threading import Thread
#from planner import tasks
from smach_ros import SimpleActionState






#condition callback for idle since it is a condition state
#TODO: setup to condition on a topic instead

def idle_cond_cb(ud):
    check = raw_input()
    if check == 1:
        return True
    else:
        return False
    

#callbacks for move, parameter reading will be moved out of the goal callback for an easier time
#(only the first and last 3 lines of that callback will remain)
def move_goal_cb(userdata,goal):
    uid=userdata.move_id
    
    params = rospy.get_param('~move'+'id')
    time = RosTime(params.time[0],params.time[1])
    velocity = Twist(Vector3(params.velocity[0],params.velocity[1],params.velocity[2]),Vector3(params.velocity[3],params.velocity[4],params.velocity[5]))
    move_goal = MoveGoal(time,velocity,theta,depth)
    move_goal = userdata.movelist[uid]
    userdata.move_id += 1
    return move_goal
#TODO: make this callback less limited and allow more arbitrary IDs
def move_result_cb(userdata,status,goal):
    if status == GoalStatus.SUCCEEDED:
        if userdata.move_id==4:
            userdata.move_id = 0
            return 'done'

def create_machine():
    static_sm = smach.StateMachine(
        outcomes=['succeeded','aborted','preempted'])
    
    with static_sm:
        #countdown = rospy.get_param('countdown')
        countdown = rospy.Time(10)
        
        smach.StateMachine.add('Idle',
                               smach_ros.ConditionState(idle_cond_cb,poll_rate=rospy.Duration(1),max_checks=1),
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
    
