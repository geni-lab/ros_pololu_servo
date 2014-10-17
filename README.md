ros_pololu_servo
================

ros catkin package for pololu servo controller board. Based on polstro 

Note:

*In case of single board Be sure to put pololu maestro in dual port mode from inside pololu provided control software. this has to be done once initially on a new pololu maestro board

*In case of daisy chain mode set the main usb connected board to usb daisy chain and the other boards in daisy chain to serial mode with 15200 baud, and unique board id's

*you should be part of dialout group, you can use >>sudo adduser $USER dialout  and then restart computer

*You need to specify yaml file with information regarding motor names, limits and ids with board id's

*run ros_pololu_servo_node

*scripts contain  pololu_trajectory_action_node.py action server to allow trajectory message based control of pololu motors
pololu_action_client_example.py is an example of how to use pololu_trajectory_action_node.py
