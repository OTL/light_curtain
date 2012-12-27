light_curtain
=================
This is Light Curtain function for ROS.
This package contains general light_curtain library and ros velocity filtering nodelet and node.

If you want to know about ROS, please see http://ros.org


velocity_curtain
-----------------
This node filters input velocity by laser scan/point cloud. If a laser scan data is detected inside robot body and the velocity is forward, then the linear.x value of output will be zero.
This is also a nodelet (light_curtain/VelocityCurtainNodelet).

Subscribed Topics
----------------

* /input_velocity (geometry_msgs/Twist) this topic is filtered by light curtain
* /curtain/scan (sensor_msgs/LaserScan) scan data for light curtain
* /curtain/points (sensor_msgs/PointCloud2) point data for light curtain
* /tf (tf/TfMessage) please publish the tf between sensor frame and robot base

Published Topics
--------------------

* /output_velocity (geometry_msgs/Twist) output velocity filtered by light curtain
* /curtain/danger_state (std_msgs/Bool) is danger state

Parameters
------------

* ~robot_width (double, default: 0.5)
* ~robot_depth (double, default: 0.5)
* ~robot_height (double, default: 1.5)
* ~keep_duration (double, default: 1.0) keep danger state for this seconds
* ~base_frame_id (string, default: base_link) tf frame name of robot base (must be on floor)

