#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
__author__ = 'drvc'

import rospy
import actionlib
import math
from ros_pololu_servo.msg import MotorCommand
#from std_msgs.msg import String

def talker():
    pub = rospy.Publisher("/pololu/command",MotorCommand,queue_size=4)
    rospy.init_node('pololu_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            position = raw_input("Enter position.")
        except ValueError:
            print("That's not an int!")
            continue
	try:
            mot_pos = int(raw_input("Witch motor?"))
        except ValueError:
            print("That's not an int!")
            continue
        #else:
        #    print("Yes an integer!")
        #    break 
        #position = raw_input("Enter M2 position.")
        #print("Prop One running @" + str(position))
        mtr = MotorCommand()
        if mot_pos == 0:
		mtr.joint_name = 'prop_one'
        elif mot_pos == 1:
		mtr.joint_name = 'prop_two'
	elif mot_pos == 2:
		mtr.joint_name = 'prop_three'
	elif mot_pos == 3:
		mtr.joint_name = 'prop_four'
	elif mot_pos == 6:
		mtr.joint_name = 'vet_one'
        elif mot_pos == 7:
		mtr.joint_name = 'vet_two'
	elif mot_pos == 8:
		mtr.joint_name = 'vet_three'
	elif mot_pos == 9:
		mtr.joint_name = 'vet_four'
	elif mot_pos == 12:
		mtr.joint_name = 'leme_one'
        elif mot_pos == 13:
		mtr.joint_name = 'leme_two'
	elif mot_pos == 14:
		mtr.joint_name = 'leme_three'
	elif mot_pos == 15:
		mtr.joint_name = 'leme_four'
	elif mot_pos == 22:
		mtr.joint_name = 'digital_one'
	elif mot_pos == 23:
		mtr.joint_name = 'digital_two'
	else:
		mtr.joint_name = 'prop_one'
		print "invalid number, motor set to prop_one"
        mtr.position = int(position)*math.pi/180
        mtr.speed = 1.0
        mtr.acceleration=1.0
        
        rospy.loginfo(mtr)
        pub.publish(mtr)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
