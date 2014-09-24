#! /usr/bin/env python
__author__ = 'mandeep'

#import roslib; roslib.load_manifest('my_pkg_name')
import rospy
import actionlib
from ros_pololu_servo.msg import *
#from trajectory_msgs.msg import JointTrajectory
#from trajectory_msgs.msg import JointTrajectoryPoint
#from ros_pololu_servo.msg import MotorCommand

class pololuTrajServer:
    def __init__(self):
        self.TimeOut=120.0
        self.pub=rospy.Publisher("/pololu/command",MotorCommand,queue_size=4)
        self.server = actionlib.SimpleActionServer('pololu_trajectory_action_server', pololu_trajectoryAction, self.execute, False)
        self.server.start()

    def moveMotor(self,jntName,pos,speed):
        mtr=MotorCommand()
        mtr.joint_name=jntName
        mtr.position=pos
        mtr.speed=speed#/self.MaxSpeed#pololu take 0 to 1.0 as speed, check the correct division
        mtr.acceleration=1.0
        self.pub.publish(mtr)

    def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
        self.jntTraj=goal.joint_trajectory#JointTrajectory()
        tme=rospy.Time.now()
        frameCount=0
        numFrames=len(self.jntTraj.points)
        r = rospy.Rate(50) # 50hz or 100hz
        processed=False
        tme=rospy.Time.now()
        while not processed:
            #process
            if frameCount>=numFrames or (rospy.Time.now()-tme>rospy.Duration().from_sec(self.TimeOut)):
                processed=True
            else:
                if frameCount<numFrames and (rospy.Time.now()-tme>self.jntTraj.points[frameCount].time_from_start):
                    nn=0
                    for pt in self.jntTraj.points[frameCount].positions:
                        self.moveMotor(self.jntTraj.joint_names[nn],pt,self.jntTraj.points[frameCount].velocities[nn])
                        nn=nn+1
                    frameCount=frameCount+1
            r.sleep()
        self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('pololu_action_server')
  server = pololuTrajServer()
  rospy.spin()
