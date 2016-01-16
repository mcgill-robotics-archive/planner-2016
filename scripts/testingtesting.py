#!/usr/bin/python

import rospy

if __name__ == '__main__':
    rospy.init_node('testingstuff',log_level=rospy.DEBUG)
    ra=rospy.get_param('~/move'+str(1))
    rospy.loginfo(ra)
    
    #server1 = InitializeServer('init_action')
    #server2 = move_server.MoveAction('square_action')
    #sm0 = smach.StateMachine(
    #    outcomes=['succeeded','aborted','preempted'])
    #sm = create_machine(sm0)
 
    #sis = smach_ros.IntrospectionServer('square_state_machine', sm, '/SM_ROOT')
    #sis.starttaken from the threading method used in auv-2015 to avoid problems
    #smach_thread = Thread(target=lambda: sm.execute())
    #smach_thread.start()

    #taken from auv-2015 to avoid the ctrl+c issue
    #rospy.on_shutdown(sm.request_preempt)
    #outcome = sm.execute()
    rospy.spin()
