#! /usr/bin/env python
# Copyright (c) 2014, OpenCog Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the OpenCog Foundation nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
