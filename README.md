ros_pololu_servo
================

ROS catkin package for the Pololu Robotics servo controller board.
Based on polstro (https://code.google.com/p/polstro/).

Notes:

* Depends on ROS package [angles](http://wiki.ros.org/angles), to install:
  ```
    user@computer:~$ sudo apt-get install ros-indigo-angles
  ```

* In the case of a single board, be sure to put pololu maestro into dual
  port mode from inside the Pololu provided control software.  This has
  to be done initially, only once, on a new Pololu Maestro board.

* When using the daisy chain mode, set the main usb connected board to
  usb daisy chain and the other boards in the daisy chain to serial mode,
  at 15200 baud, and unique board id's

* You need to be a part of dialout group. You can set this with
  ```
    user@computer:~$ sudo adduser $USER dialout
  ```
  and then logout and log back in.  Alternately, it should be enough
  close all terminal windows, and reopen them.

* You need to specify a yaml file with information regarding motor
  names, limits and ids with board id's.

* run ros_pololu_servo_node

* Scripts should include the pololu_trajectory_action_node.py action
  server to allow trajectory-message-based control of Pololu motors.
  An example of how to use this can be found in the 
  pololu_action_client_example.py file.


HOWTO
-----
Here are examples of:

 * [how to program it](https://github.com/geni-lab/hri_common/blob/master/scripts/mini_head_lip_sync_node.py)

 * [how to launch it](https://github.com/geni-lab/zoidstein_robot/blob/master/zoidstein_driver/launch/zoidstein_driver.launch)

 * [how to write a config file](https://github.com/geni-lab/zoidstein_robot/blob/master/zoidstein_driver/launch/zoidstein_pololu_motors.yaml)
