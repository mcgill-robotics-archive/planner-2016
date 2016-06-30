McGill Robotics Planner
=======================
How to run the programs:

From inside planner(roscd planner), roslaunch launch/planner.launch, then in another tab, use rostopic pub /sm_reset std_msgs/Bool True                
(or False) , this simulates flipping the switch/button (True for Running, False/no signal for stopping/going back to idle).

Issues:
-The ctr+c preempt doesn't work even using the 'trick', so use ctrl+z and then
kill -9 \`jobs -ps\` to remove all stopped processes if they start causing issues.

-The only thing preventing this from 100% working is a weird concurrency thing where it tries to return an invalid transition.
It's as if it thinks the sequence inside the second monitor is also a monitor. Oddly enough, it spits a nasty looking error
but the planner then goes on its merry way and runs seemingly just fine.

-I'm not sure the monitor preempts the actions quickly enough (potentially a big problem).Will try something with it.

-Roslauch works for loading the params but it chokes on the previously mentioned concurrency problem. Launch the
script directly (./scripts/planner_all) to see it better

-yaml not loading in correct order, maybe params is indeed not the best way to load it. Will try how Jana loaded it.

Quick wiki:

The planner uses ActionStates to send goals and receive responses to and from taskr action servers
(and potentially other kinds of services could be done through service states too).
An action state is an action client in the form of a smach state.

A smach state is created with information about it and linked to other states through
transitions. All the states here are inside what is called concurrence conatainers, which monitor
the kill switch to reset the robot back into idle mode.

To generate the states, you need parameters from the param server, which are currently sent
from a yaml file. Current challenge is fixing the weird concurrency issue.
