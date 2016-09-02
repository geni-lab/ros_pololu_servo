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

#include <ros_pololu_servo/PololuController.h>
#include <ros_pololu_servo/PololuMath.h>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <ros_pololu_servo/MotorRange.h>
#include <algorithm>

using namespace ros_pololu_servo;
using namespace std;
using namespace angles;


PololuController::PololuController()
{
    int x = 0;
}

PololuController::~PololuController()
{
    ROS_INFO("Shutting down...");
    delete serial_interface;
    serial_interface = NULL;
}

bool PololuController::initialize()
{
    ros::NodeHandle nh("~");

    bool success = true;

    // Load parameters
    ROS_INFO("Loading pololu_motors_yaml...");
    // Load named
    if (nh.hasParam("pololu_motors_yaml"))
    {
        nh.getParam("pololu_motors_yaml", pololu_config_dir);
        PololuYamlParser::parse(pololu_config_dir, motors);
    }
    nh.param<string>("port_name", port_name, "/dev/ttyACM0");
    nh.param<int>("baud_rate", baud_rate, 115200);
    nh.param<int>("rate_hz", rate_hz, 10);
    nh.param<bool>("daisy_chain", daisy_chain, false);
    nh.param<string>("topic_prefix", topic_prefix, "pololu/");
    nh.param<string>("topic_name", topic_name, "command");
    // Create serial interface
    serial_interface = Polstro::SerialInterface::createSerialInterface(port_name, baud_rate);
    if (!serial_interface->isOpen())
    {
        ROS_ERROR("Failed to open interface, exiting");
        success = false;
    }

    // Setup publisher and subscriber
    motor_state_list_pub = n.advertise<MotorStateList>("pololu/motor_states", 10);
    motor_cmd_sub = n.subscribe(topic_prefix+topic_name, 50, &PololuController::motor_command_callback, this);

    // Setup services
    motor_range_srv = n.advertiseService(topic_prefix+"motor_range", &PololuController::motor_range_callback, this);

    return success;
}

bool PololuController::motor_range_callback(MotorRange::Request &req, MotorRange::Response &res)
{
    ROS_INFO("Recevied motor_range_callback for motor: %s", req.motor_name.c_str());
    map<string, Motor>::iterator iterator = motors.find(req.motor_name);

    if(iterator != motors.end())
    {
        Motor motor = motors[req.motor_name];
        res.min = PololuMath::to_radians(motor.min, motor);
        res.max = PololuMath::to_radians(motor.max, motor);
        res.direction = motor.direction;
        return true;
    }
    else
    {
        ROS_ERROR("motor %s hasn't been loaded into pololu_config", req.motor_name.c_str());
        return false;
    }
}

void PololuController::publish_motor_state()
{

    //refresh the motor states
    motor_state_list.motor_states.clear();

    //iterate through all of the motors
    for(map<string, Motor>::iterator iterator = motors.begin(); iterator != motors.end(); iterator++)
    {
        unsigned short pulse;
        MotorState motor_state;

        Motor motor;
        motor = (iterator->second);

        //nab the pulse from the interface based on the motor_id we're requesting about.
        if(daisy_chain)
        {
            serial_interface->getPositionPP(motor.pololu_id, motor.motor_id, pulse);
        }
        else
        {
            serial_interface->getPositionCP(motor.motor_id, pulse);
        }

       //he's dividing by four to convert Maestro's PWM pulse to what we're working with.
        pulse = pulse * 0.25;


        motor_state.name = motor.name;
        motor_state.pololu_id = motor.pololu_id;
        motor_state.motor_id = motor.motor_id;

        //yay, conversion operations
        motor_state.radians = PololuMath::to_radians(pulse, motor) * motor.direction;
        motor_state.degrees = to_degrees(motor_state.radians); // (this looks like an error): * motor.direction;
        motor_state.pulse = pulse;

        motor_state.calibration.min_pulse = motor.calibration.min_pulse;
        motor_state.calibration.min_radians = PololuMath::to_radians(motor.calibration.min_pulse, motor);
        motor_state.calibration.min_degrees = to_degrees(motor_state.calibration.min_radians);

        motor_state.calibration.max_pulse = motor.calibration.max_pulse;
        motor_state.calibration.max_radians = PololuMath::to_radians(motor.calibration.max_pulse, motor);
        motor_state.calibration.max_degrees = to_degrees(motor_state.calibration.max_radians);

        motor_state_list.motor_states.push_back(motor_state);
    }

    motor_state_list_pub.publish(motor_state_list);
}

double PololuController::get_rate_hz()
{
    return rate_hz;
}

Motor PololuController::default_motor(){
  Motor m;
  m.pololu_id = 0;
  m.motor_id = -1;
  m.init = 1500;
  m.min = 820;
  m.max = 2175;
  m.direction = 1.0;
  m.calibration.min_pulse = 820;
  m.calibration.max_pulse = 2175;
  m.calibration.min_angle = -M_PI/2;
  m.calibration.max_angle = M_PI/2;
  return m;
}

void PololuController::motor_command_callback(const MotorCommand::ConstPtr& msg)
{
    ROS_INFO("Recevied cmd name: %s, position: %f, speed: %f, accel: %f", msg->joint_name.c_str(), to_degrees(msg->position), msg->speed, msg->acceleration);

    map<string, Motor>::iterator iterator = motors.find(msg->joint_name);
    // Allow to send commands to named and unnamed motors
    bool success = true;
    Motor motor;
    if(iterator != motors.end()){
      motor = motors[msg->joint_name];
    }else{
      int motor_id = std::atoi(msg->joint_name.c_str());

      if (motor_id == 0 && msg->joint_name != "0" ){
        success = false;
      }else{
        motor = default_motor();
        motor.motor_id = motor_id;
      }
    }
    if(success)
    {

        bool send_commands = true;

        if(msg->speed < 0.0 || 1.0 < msg->speed)
        {
            send_commands = false;
            ROS_ERROR("trying to set motor %s speed to %f (should be between 0.0 - 1.0)", msg->joint_name.c_str(), msg->speed);
        }

        if(msg->acceleration < 0.0 || 1.0 < msg->acceleration)
        {
            send_commands = false;
            ROS_ERROR("trying to set motor %s acceleration is %f (should be between 0.0 - 1.0)", msg->joint_name.c_str(), msg->acceleration);
        }

        double min = PololuMath::to_radians(motor.min, motor);
        double max = PololuMath::to_radians(motor.max, motor);
        double new_min = std::min(min, max);
        double new_max = std::max(min, max);
        double new_position = msg->position;


        if((new_min - 0.001) > new_position)
        {
            ROS_ERROR("trying to set motor %s position is %f degrees (should be between %f and %f degrees. Attempting to fail gracefully by clamping.)", msg->joint_name.c_str(), msg->position, new_min, new_max);
            new_position = new_min + 0.001;
        }
        if((new_max + 0.001) < new_position)
        {
            ROS_ERROR("trying to set motor %s position is %f degrees (should be between %f and %f degrees. Attempting to fail gracefully by clamping.)", msg->joint_name.c_str(), msg->position, new_min, new_max);
            new_position = new_max - 0.001;
        }

        if(send_commands)
        {
            double pulse = PololuMath::to_pulse(new_position, motor);

            double speed = PololuMath::interpolate(msg->speed, 0.0, 1.0, 0, 255.0); //Set speed, make sure doesn't below 0, which is max speed
            double acceleration = PololuMath::interpolate(msg->acceleration, 0.0, 1.0, 0, 255.0); //Set acceleration, make sure doesn't go below 0, which is max acceleration
            double pulse_m = PololuMath::clamp(pulse * 4.0, 3280, 8700);

            if(daisy_chain)
            {
                serial_interface->setSpeedPP(motor.pololu_id, motor.motor_id, speed);
                serial_interface->setAccelerationPP(motor.pololu_id, motor.motor_id, acceleration);
                serial_interface->setTargetPP(motor.pololu_id, motor.motor_id, (int)pulse_m);
                ROS_INFO("id: %d/%d, pulse:  %f, pos: %f, speed: %f, accel: %f", motor.pololu_id, motor.motor_id, pulse_m, msg->position, speed, acceleration);
            }
            else
            {
                serial_interface->setSpeedCP(motor.motor_id, speed);
                serial_interface->setAccelerationCP(motor.motor_id, acceleration);
                serial_interface->setTargetCP(motor.motor_id, (int)pulse_m);
                ROS_INFO("id: %d, pulse:  %f, pos: %f, speed: %f, accel: %f", motor.motor_id, pulse_m, msg->position, speed, acceleration);
            }
        }
    }
    else
    {
        ROS_ERROR("motor %s hasn't been loaded into pololu_config", msg->joint_name.c_str());
    }
}

