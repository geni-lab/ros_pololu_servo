#include <ros/ros.h>
#include <vector>
#include <ros_pololu_servo/MotorCommand.h>
#include <ros_pololu_servo/MotorState.h>
#include <ros_pololu_servo/MotorStateList.h>
#include <math.h>
#include <map>
#include "PolstroSerialInterface.h"
#include "yaml-cpp/yaml.h"

using namespace ros_pololu_servo;
using namespace std;

ros::Publisher pub;
YAML::Node config;
Polstro::SerialInterface* serial_interface;
string pololu_config_dir, port_name, pololu_namespace;
int baud_rate, channel_min_value, channel_max_value, channel_value_range, rate_hz;
MotorStateList motor_state_list;
map<string, int> joints;

void publish_motor_state()
{
	motor_state_list.motor_states.clear();

	for (std::size_t i = 0; i < config.size(); i++)
	{
        unsigned char channel_number = atoi(config[i]["id"].as<string>().c_str());
        unsigned short position;

        serial_interface->getPositionCP(channel_number, position);
        float angle = (((float)(position - channel_min_value) / (float)channel_max_value) - 0.5) * M_PI;

        ROS_INFO("channel_number: %d, position: %d", channel_number, position);
        MotorState motor;
        motor.id = channel_number;
        motor.name = config[i]["name"].as<string>();
        motor.position = angle;
        motor_state_list.motor_states.push_back(motor);
	}

	pub.publish(motor_state_list);
}

void motor_command_callback(const MotorCommand::ConstPtr& msg)
{
    int channel_number = joints[msg->joint_name];

	if ((msg->position >= -M_PI/2) && (msg->position <= M_PI/2))
	{
		int i = ((msg->position + M_PI/2.0) / M_PI) * (float)channel_value_range + (float)channel_min_value;
		ROS_INFO("joint_name: %s, id: %d, position: %d", msg->joint_name.c_str(), channel_number, i);

		if (i >= channel_min_value && i<= channel_max_value)
		{
		    serial_interface->setSpeedCP(channel_number, msg->speed);
            serial_interface->setAccelerationCP(channel_number, msg->acceleration);
		    serial_interface->setTargetCP(channel_number, i);
        }
	}
}

int main(int argc,char**argv)
{
	ros::init(argc, argv, "pololu_servo", ros::init_options::AnonymousName);
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

    // Load parameters
	ROS_INFO("Loading parameters");

	if (nh.hasParam("pololu_config"))
    {
        nh.getParam("pololu_config", pololu_config_dir);
        config = YAML::LoadFile(pololu_config_dir);

        for (std::size_t i = 0; i < config.size(); i++)
	    {
	        string joint_name = config[i]["name"].as<string>();
	        int channel_number = i;
	        joints[joint_name] = channel_number;
        }
    }
    else
    {
        ROS_ERROR("pololu_config file not specified, exiting");
        return EXIT_FAILURE;
    }

    nh.param<string>("pololu_namespace", pololu_namespace, "pololu");
    nh.param<string>("port_name", port_name, "/dev/ttyACM0");
    nh.param<int>("baud_rate", baud_rate, 115200);
    nh.param<int>("channel_min_value", channel_min_value, 4000);
    nh.param<int>("channel_max_value", channel_max_value, 8000);
    nh.param<int>("rate_hz", rate_hz, 10);
    channel_value_range = channel_max_value - channel_min_value;

    // Create serial interface
	ROS_INFO("Creating serial interface '%s' at %d bauds\n", port_name.c_str(), baud_rate);
	serial_interface = Polstro::SerialInterface::createSerialInterface(port_name, baud_rate);
	if (!serial_interface->isOpen())
	{
		ROS_ERROR("Failed to open interface, exiting");
		return EXIT_FAILURE;
	}

    // Setup publisher and subscriber
	pub = n.advertise<MotorStateList>(pololu_namespace + "/motor_states", 100);
	n.subscribe(pololu_namespace + "/motor_command", 20, motor_command_callback);

    // Publish motor state
    ROS_INFO("Ready...");
    ros::Rate rate(rate_hz);
	while(ros::ok())
	{
	    publish_motor_state();
		ros::spinOnce();
        rate.sleep();
	}

	ROS_INFO("Deleting serial interface...");
	delete serial_interface;
	serial_interface = NULL;
	return EXIT_SUCCESS;
}