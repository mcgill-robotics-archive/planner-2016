McGill Robotics Planner
=======================
How to run the programs:

Run them, then in another tab, use rostopic pub /sm_reset std_msgs/Bool True                
(or False) , this simulates flipping the switch/button (True for Running, False/no signal for stopping/going back to idle).

Issues:
-The ctr+c preempt doesn't work even using the 'trick', so use ctrl+z and then
kill -9 `jobs -ps` to remove all stopped processes if they start causing issues.

-Roslauch doesn't work (yaml file doesn't load automatically). Use
rosparam load with the path to square.yaml (it's inside config) to get it on the param server,
program works fine with the params once that is done.

-The calculations for move don't mean anything.

Quick wiki:

The planner uses ActionStates to send goals and receive responses to and from taskr action servers
(and potentially other kinds of services could be done through service states too).
An action state is an action client in the form of a smach state.

A smach state is created with information about it and linked to other states through
transitions. All the states here are inside what is called concurrence conatainers, which monitor
the kill switch to reset the robot back into idle mode.

To generate the states, you need parameters from the param server, which are currently sent
from a yaml file. The challenge now is the possibility of having to generate parameters for a state on the fly.
