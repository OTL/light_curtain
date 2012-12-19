light_curtain
=================


velocity_filter
-------------
This node filters input velocity by laser scan. If the laser is inside of robot body, the output is not published.

Subscribed Topics
----------------

* /input_velocity (geometry_msgs/Twist) this topic is filtered by light curtain
* /scan (sensor_msgs/LaserScan) data for light curtain
* /tf (tf/TfMessage) please publish laser's tf

Published Topics
--------------------

* /cmd_vel (geometry_msgs/Twist) output


Parameters
------------

* ~robot_width (double, default: 0.5)
* ~robot_depth (double, default: 0.5)
* ~robot_height (double, default: 1.0)
* ~base_frame (string, default: base_link) tf frame name of robot base (must be on floor)
