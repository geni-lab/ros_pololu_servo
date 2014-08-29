#include "PololuController.h"
#include "PololuMath.h"
#include <string>
#include <ros/ros.h>
#include <ros_pololu_servo/MotorRange.h>

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

	if (nh.hasParam("pololu_motors_yaml"))
    {
        nh.getParam("pololu_motors_yaml", pololu_config_dir);
        success = PololuYamlParser::parse(pololu_config_dir, motors);
    }
    else
    {
        ROS_ERROR("pololu_config file not specified, exiting");
        success = false;
    }

    nh.param<string>("port_name", port_name, "/dev/ttyACM0");
    nh.param<int>("baud_rate", baud_rate, 115200);
    nh.param<int>("rate_hz", rate_hz, 10);

    // Create serial interface

	serial_interface = Polstro::SerialInterface::createSerialInterface(port_name, baud_rate);
	if (!serial_interface->isOpen())
	{
		ROS_ERROR("Failed to open interface, exiting");
		success = false;
	}

    // Setup publisher and subscriber
	motor_state_list_pub = n.advertise<MotorStateList>("pololu/motor_states", 10);
	motor_cmd_sub = n.subscribe("pololu/command", 10, &PololuController::motor_command_callback, this);

	// Setup services
	motor_range_srv = n.advertiseService("pololu/motor_range", &PololuController::motor_range_callback, this);

	return success;
}

bool PololuController::motor_range_callback(MotorRange::Request &req, MotorRange::Response &res)
{
    ROS_INFO("Recevied motor_range_callback for motor: %s", req.motor_name.c_str());
    map<string, Motor>::iterator iterator = motors.find(req.motor_name);

    if(iterator != motors.end())
    {
        Motor motor = motors[req.motor_name];
        res.min = motor.min;
        res.max = motor.max;
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
	motor_state_list.motor_states.clear();


	//ROS_INFO("Starting");

	for(map<string, Motor>::iterator iterator = motors.begin(); iterator != motors.end(); iterator++)
	{

	    Motor motor;
	    motor = (iterator->second);

        //ROS_INFO("Bitch");

        unsigned short pulse;
        serial_interface->getPositionCP(motor.motor_id, pulse);

        //ROS_INFO("Pulse: %d", pulse);

        MotorState motor_state;
        motor_state.name = motor.name;
        motor_state.pololu_id = motor.pololu_id;
        motor_state.motor_id = motor.motor_id;
        motor_state.pulse = pulse;


        double angle = PololuMath::pulse_to_angle(pulse, motor.calibration);
        motor_state.radians = PololuMath::to_joint_pos(angle, motor); //, motor); //PololuMath::to_joint_pos(;
        motor_state.degrees = to_degrees(motor_state.radians);
        motor_state_list.motor_states.push_back(motor_state);
    }

	motor_state_list_pub.publish(motor_state_list);

	//ROS_INFO("Published!");
}

double PololuController::get_rate_hz()
{
    return rate_hz;
}

void PololuController::motor_command_callback(const MotorCommand::ConstPtr& msg)
{
    ROS_INFO("Recevied cmd name: %s, position: %f, speed: %f, accel: %f", msg->joint_name.c_str(), to_degrees(msg->position), msg->speed, msg->acceleration);

    map<string, Motor>::iterator iterator = motors.find(msg->joint_name);

    if(iterator != motors.end())
    {
        Motor motor = motors[msg->joint_name];
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

        if((msg->position < motor.min && !PololuMath::are_same(motor.min, msg->position)) || (motor.max < msg->position && !PololuMath::are_same(motor.max, msg->position)))
        {
            send_commands = false;
            ROS_ERROR("trying to set motor %s position is %f degrees (should be between %f and %f degrees)", msg->joint_name.c_str(), to_degrees(msg->position), to_degrees(motor.min), to_degrees(motor.max));
        }

        if(send_commands)
        {
            double angle = PololuMath::to_motor_pos(msg->position, motor);
            int pulse = PololuMath::angle_to_pulse(angle, motor.calibration);
            double speed = PololuMath::interpolate(msg->speed, 0.0, 1.0, 1.0, 255.0); //Set speed, make sure doesn't below 1, which is max speed
            double acceleration = PololuMath::interpolate(msg->acceleration, 0.0, 1.0, 1.0, 255.0); //Set acceleration, make sure doesn't go below 1, which is max acceleration

            ROS_INFO("id: %d/%d, pulse:  %d, pos: %f, speed: %f, accel: %f", motor.pololu_id, motor.motor_id, pulse, to_degrees(angle), speed, acceleration);

            serial_interface->setSpeedCP(motor.motor_id, speed);
            serial_interface->setAccelerationCP(motor.motor_id, acceleration);
            serial_interface->setTargetCP(motor.motor_id, pulse);
        }
    }
    else
    {
        ROS_ERROR("motor %s hasn't been loaded into pololu_config", msg->joint_name.c_str());
    }
}

