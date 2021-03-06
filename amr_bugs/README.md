Summary
=======

This package provides a family of obstacle avoidance algorithms.

The launch file "*wallfollower.launch*" brings up the Stage simulator with
"*Rooms*" world (which is well-suited for testing wallfollowing behavior),
the `wallfollower.py` node, dynamic reconfigure GUI, and [SMACH introspection
GUI](http://www.ros.org/wiki/smach_viewer).

The launch file "*bug2.launch*" brings up the Stage simulator with "*Walls*"
world (which is well-suited for testing bug2 behavior), the `bug2.py`,
`wallfollower.py`, and `motioncontroller` nodes, and the MoveTo action client
GUI (set up to send commands to the `bug2.py` node).

Nodes
=====

wallfollower.py
---------------

Wallfollower implements the simplest obstacle avoidance behavior by simply
moving along the first wall (obstacle) it happened to face. It is implemented
as a [SMACH](http://www.ros.org/wiki/smach) state machine and provides services
to start and stop itself. It relies on the readings of the Pioneer sonars.

This node supports dynamic reconfiguration during runtime. The desired distance
to the wall (`clearance`) and the wallfollowing mode (left or right) could be
adjusted through the `reconfigure_gui`.

### Subscribed topics

* `sonar_pioneer` (*amr_msgs/Ranges*)  
  range readings from the pioneer sonar ring

### Published topics

* `cmd_vel` (*geometry_msgs/Twist*)  
  velocity commands to control the robot

### Parameters

* `~enable_at_startup` (default: true)  
  controls whether the node should enable the behavior after startup

### Services provided

* `~enable` (*std_srvs/Empty*)  
  start the wallfollowing behavior

* `~disable` (*std_srvs/Empty*)  
  stop the wallfollowing behavior


[actionlib]: http://www.ros.org/wiki/actionlib
