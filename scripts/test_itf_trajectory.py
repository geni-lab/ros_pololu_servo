#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

names=['motor_one','motor_two']

if __name__ == '__main__':
    rospy.init_node('pololu_trajectory_example', anonymous=True)
    pub=rospy.Publisher("/pololu_trajectory", JointTrajectory,queue_size=4)
    rate=rospy.Rate(0.25)
    for n in range (1,4):
        print "trajectory=" + str(n)
        traj=JointTrajectory()
        traj.header.stamp=rospy.Time.now()
        traj.joint_names.append(names[0])
        traj.joint_names.append(names[1])
        pts=JointTrajectoryPoint()
        pts.time_from_start=rospy.Duration(0.5)
        pts.positions.append(-0.77)
        pts.positions.append(0.77)
        pts.velocities.append(1.0)
        pts.velocities.append(1.0)
        traj.points.append(pts)
        pts=JointTrajectoryPoint()
        pts.time_from_start=rospy.Duration(1.0)
        pts.positions.append(0.77)
        pts.positions.append(-0.77)
        pts.velocities.append(1.0)
        pts.velocities.append(1.0)
        traj.points.append(pts)
        pub.publish(traj)
        rate.sleep()
