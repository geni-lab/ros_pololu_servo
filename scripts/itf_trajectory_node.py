#!/usr/bin/env python

import rospy
#from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ros_pololu_servo.msg import MotorCommand

class trajectory:
    #strin=""

    def callback(self,data):
        #rospy.loginfo(rospy.get_caller_id()+"******I heard*******,%s",data.data)
        #self.strin=data.data
        print("callback")
        if not self.rcvd:
            self.rcvTraj=data
            self.rcvd=True
            print("recvd")

    def __init__(self):

        rospy.init_node('pololu_trajectory_listener', anonymous=True)

        self.MaxSpeed=6.2 #radians/sec
        self.TimeOut=120.0 #seconds

        self.rcvTraj=JointTrajectory()
        self.rcvd=False
        self.processing=False

        self.pub=rospy.Publisher("/pololu/command",MotorCommand,queue_size=4)
        rospy.Subscriber("/pololu_trajectory", JointTrajectory, self.callback)
        #rospy.Subscriber("pololu_trajectory", String, self.callback)

    def moveMotor(self,jntName,pos,speed):
        mtr=MotorCommand()
        mtr.joint_name=jntName
        mtr.position=pos
        mtr.speed=speed#/self.MaxSpeed#pololu take 0 to 1.0 as speed, check the correct division
        mtr.acceleration=1.0
        self.pub.publish(mtr)

    def start(self):
        self.rcvd=False
        self.processing=False
        self.jntTraj=JointTrajectory()
        tme=rospy.Time.now()
        frameCount=0
        numFrames=0
        r = rospy.Rate(50) # 50hz or 100hz
        while not rospy.is_shutdown():
            if self.rcvd and not self.processing:
                #copy rcvd buffer
                self.jntTraj=self.rcvTraj#how to make deep copy? will work if not updated till done
                self.processing=True
                tme=rospy.Time.now()
                frameCount=0
                numFrames=len(self.jntTraj.points)

            if self.processing:#added self.recvd
                #process
                if frameCount>=numFrames or (rospy.Time.now()-tme>rospy.Duration().from_sec(self.TimeOut)):
                    self.processing=False
                    self.rcvd=False
                else:
                    if frameCount<numFrames and (rospy.Time.now()-tme>self.jntTraj.points[frameCount].time_from_start):
                        nn=0
                        for pt in self.jntTraj.points[frameCount].positions:
                            self.moveMotor(self.jntTraj.joint_names[nn],pt,self.jntTraj.points[frameCount].velocities[nn])
                            nn=nn+1
                        frameCount=frameCount+1
            r.sleep()

if __name__ == '__main__':
    traj=trajectory()
    traj.start()
