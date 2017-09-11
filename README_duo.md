TeraRanger Duo ROS Module
=========================

This is the ROS module for the TeraRanger Duo ranging sensor (www.teraranger.com).


Using module
============

To use the ROS node you just need to:
* Create a ROS Workspace
* Copy the node terarangerduo package into the workspace src directory
* Compile using: catkin_make 
* Run using: rosrun terarangerduo terarangerduo_node _portname:=/dev/ttyUSB0

If you want to change the operating mode, run
* rosrun rqt_reconfigure rqt_reconfigure 

NB: remember to execute the daemon roscore before running the rosrun command
