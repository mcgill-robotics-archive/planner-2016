import rospy
import smach
import actionlib
from smach_ros import SimpleActionState



#class Initialize(SimpleActionState):
    #def _init_(countdown,offset,time_left):
        
    

#class Move(SimpleActionState):

def idle_cond_cb(ud):
    check = raw_input()
    if check == 1:
        return True
    else:
        return False
    


def move_goal_cb(userdata,goal):
    uid=userdata.move_id
    
    params = rospy.getparams('~move'+'id')
    time = RosTime(params.time[0],params.time[1])
    velocity = Twist(Vector3(params.velocity[0],params.velocity[1],params.velocity[2]),Vector3(params.velocity[3],params.velocity[4],params.velocity[5]))
    move_goal = MoveGoal(time,velocity,theta,depth)
    move_goal = userdata.movelist[uid]
    userdata.move_id += 1
    return move_goal

def move_result_cb(userdata,status,goal):
    if status == GoalStatus.SUCCEEDED:
        if userdata.move_id==rospy.getparams('~numberofmoves'):
            userdata.move_id = 0
            return 'done'
    
    
        
