#pragma once

#include <ros/ros.h>
#include <vector>
#include <map>
#include <angles/angles.h>
#include <string>
#include "PolstroSerialInterface.h"
#include "PololuYamlParser.h"
#include "Motor.h"
#include "Calibration.h"
#include "PololuController.h"
#include <ros_pololu_servo/MotorCommand.h>
#include <ros_pololu_servo/MotorState.h>
#include <ros_pololu_servo/MotorStateList.h>

struct Motor;

class PololuController
{
    private:
        ros::Publisher motor_state_list_pub;
        ros::Subscriber motor_cmd_sub;
        ros::NodeHandle n;
        //ros::Rate rate;

        Polstro::SerialInterface* serial_interface;
        std::string pololu_config_dir, port_name;
        int baud_rate, rate_hz;

        ros_pololu_servo::MotorStateList motor_state_list;
        map<string, Motor> motors;

    public:
        PololuController();
        ~PololuController();
        bool initialize();
        double get_rate_hz();
        void publish_motor_state();
        void motor_command_callback(const ros_pololu_servo::MotorCommand::ConstPtr& msg);
};
