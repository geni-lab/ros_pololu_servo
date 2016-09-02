/* Copyright (c) 2014, OpenCog Foundation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the OpenCog Foundation nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <ros/ros.h>
#include <vector>
#include <map>
#include <angles/angles.h>
#include <string>
#include <polstro/PolstroSerialInterface.h>
#include <ros_pololu_servo/PololuYamlParser.h>
#include <ros_pololu_servo/Motor.h>
#include <ros_pololu_servo/Calibration.h>
#include <ros_pololu_servo/PololuController.h>
#include <ros_pololu_servo/MotorCommand.h>
#include <ros_pololu_servo/MotorState.h>
#include <ros_pololu_servo/MotorStateList.h>
#include <ros_pololu_servo/MotorRange.h>

struct Motor;

class PololuController
{
    private:
        ros::Publisher motor_state_list_pub;
        ros::Subscriber motor_cmd_sub;
        ros::ServiceServer motor_range_srv; // = n.advertiseService("add_two_ints", add);
        ros::NodeHandle n;
        //ros::Rate rate;

        Polstro::SerialInterface* serial_interface;
        std::string pololu_config_dir, port_name, topic_prefix, topic_name;
        int baud_rate, rate_hz;
        bool daisy_chain;

        ros_pololu_servo::MotorStateList motor_state_list;
        map<string, Motor> motors;
        Motor default_motor();

    public:
        PololuController();
        ~PololuController();
        bool motor_range_callback(ros_pololu_servo::MotorRange::Request &req, ros_pololu_servo::MotorRange::Response &res);
        bool initialize();
        double get_rate_hz();
        void publish_motor_state();
        void motor_command_callback(const ros_pololu_servo::MotorCommand::ConstPtr& msg);
};
