#! /usr/bin/env python
__author__ = 'mandeep'

import rospy
import actionlib

from ros_pololu_servo.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint

names=['motor_one','motor_two']

if __name__ == '__main__':
    rospy.init_node('pololu_action_example_client')
    client = actionlib.SimpleActionClient('pololu_trajectory_action_server', pololu_trajectoryAction)
    client.wait_for_server()

    goal = pololu_trajectoryGoal()
    traj=goal.joint_trajectory
    traj.header.stamp=rospy.Time.now()
    traj.joint_names.append(names[0])
    traj.joint_names.append(names[1])
    pts=JointTrajectoryPoint()
    pts.time_from_start=rospy.Duration(2.0)
    pts.positions.append(-0.77)
    pts.positions.append(0.77)
    pts.velocities.append(1.0)
    pts.velocities.append(1.0)
    traj.points.append(pts)
    pts=JointTrajectoryPoint()
    pts.time_from_start=rospy.Duration(3.0)
    pts.positions.append(0.77)
    pts.positions.append(-0.77)
    pts.velocities.append(1.0)
    pts.velocities.append(1.0)
    traj.points.append(pts)

    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
