#include <ros/ros.h>
#include <vector>
#include <ros_pololu_servo/MotorCommand.h>
#include <ros_pololu_servo/MotorState.h>
#include <ros_pololu_servo/MotorStateList.h>
#include <math.h>
#include <map>
#include "PolstroSerialInterface.h"
#include "yaml-cpp/yaml.h"
#include <math.h>

using namespace ros_pololu_servo;
using namespace std;

struct Motor
{
    int id;
    string name;
    double min;
    double init;
    double max;
    double direction;
};

//neck_yaw_joint:
//  id: 1
//  min_angle: -30
//  center_angle: 0.0
//  max_angle: 30

double deg_to_rad(double degrees)
{
    return degrees * (M_PI / 180.0);
}

double rad_to_deg(double radians)
{
    return radians * (180.0 / M_PI);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double to_motor_pos(double value, Motor motor)
{
    return (value + motor.init) * motor.direction;
}

double to_joint_pos(double value, Motor motor)
{
    return (value + motor.init) * motor.direction;
}


namespace YAML
{
    template<>
    struct convert<Motor>
    {
        static bool decode(const Node& node, Motor& motor)
        {
            if(!node.IsMap())
            {
                printf("Error, motor does not contain key value pairs, i.e. id, min, init, max and reverse\n");
                return false;
            }
            else if(!node["id"])
            {
                printf("Error, motor id not specified\n");
                return false;
            }
            else if(!node["min"])
            {
                printf("Error, motor min not specified\n");
                return false;
            }
            else if(!node["init"])
            {
                printf("Error, motor init not specified\n");
                return false;
            }
            else if(!node["max"])
            {
                printf("Error, motor max not specified\n");
                return false;
            }
            else if(!node["reverse"])
            {
                printf("Error, motor reverse not specified\n");
                return false;
            }

            motor.id = node["id"].as<int>();
            motor.min = deg_to_rad(node["min"].as<double>());
            motor.init = deg_to_rad(node["init"].as<double>());
            motor.max = deg_to_rad(node["max"].as<double>());
            motor.direction = 1.0;

            if(node["reverse"].as<bool>())
            {
                motor.direction = -1.0;
            }

            return true;
        }
    };
}


ros::Publisher pub;
Polstro::SerialInterface* serial_interface;
string pololu_config_dir, port_name, pololu_namespace;
int baud_rate, pwm_value_range, rate_hz;
int pwm_min_value = 4000;
int pwm_max_value = 8000;
MotorStateList motor_state_list;
map<string, Motor> motors;
float EPSILON = 0.001;

double interpolate(double value, double old_min, double old_max, double new_min, double new_max)
{
    // Width of each range
    double old_range = old_max - old_min;
    double new_range = new_max - new_min;

    // Scale old value into range between 0 and 1
    double scaled_value = double(value - old_min) / double(old_range);

    // Convert the scaled value into the new range
    double new_val = new_min + (scaled_value * new_range);

    printf("old_range: %f, new_range: %f, scaled_val: %f, new_val: %f", old_range, new_range, scaled_value, new_val);

    return new_val;
}

void publish_motor_state()
{
	motor_state_list.motor_states.clear();

	for(map<string, Motor>::iterator iterator = motors.begin(); iterator != motors.end(); iterator++)
	{
	    Motor motor;
	    motor = (iterator->second);

        unsigned short pulse;
        serial_interface->getPositionCP(motor.id, pulse);
        double position = (((double)(pulse - pwm_min_value) / (double)pwm_value_range) - 0.5) * M_PI;

        MotorState motor_state;
        motor_state.id = motor.id;
        motor_state.name = motor.name;
        motor_state.position = to_joint_pos(position, motor);
        motor_state.position_deg = rad_to_deg(to_joint_pos(position, motor));
        motor_state_list.motor_states.push_back(motor_state);
    }

	pub.publish(motor_state_list);
}

bool are_same(double a, double b)
{
    return fabs(a - b) < EPSILON;
}

void motor_command_callback(const MotorCommand::ConstPtr& msg)
{
    ROS_INFO("Recevied cmd name: %s, position: %f, speed: %f, accel: %f", msg->joint_name.c_str(), rad_to_deg(msg->position), msg->speed, msg->acceleration);

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

        if((msg->position < motor.min && !are_same(motor.min, msg->position)) || (motor.max < msg->position && !are_same(motor.max, msg->position)))
        {
            send_commands = false;
            ROS_ERROR("trying to set motor %s position is %f degrees (should be between %f and %f degrees)", msg->joint_name.c_str(), rad_to_deg(msg->position), rad_to_deg(motor.min), rad_to_deg(motor.max));
        }

        if(send_commands)
        {
            unsigned char motor_id = motor.id;

            int pulse = ((to_motor_pos(msg->position, motor) + M_PI/2.0) / M_PI) * (double)pwm_value_range + (double)pwm_min_value;
            double speed_cp = interpolate(msg->speed, 0.0, 1.0, 1.0, 255.0); //Set speed, make sure doesn't below 1, which is max speed
            double acceleration_cp = interpolate(msg->acceleration, 0.0, 1.0, 1.0, 255.0); //Set acceleration, make sure doesn't go below 1, which is max acceleration

            ROS_INFO("id: %d, pos: %f, speed: %f, accel: %f", motor.id, rad_to_deg(to_motor_pos(msg->position, motor)), speed_cp, acceleration_cp);

            serial_interface->setSpeedCP(motor_id, speed_cp);
            serial_interface->setAccelerationCP(motor_id, acceleration_cp);
            serial_interface->setTargetCP(motor_id, pulse);
        }
    }
    else
    {
        ROS_ERROR("motor %s hasn't been loaded into pololu_config", msg->joint_name.c_str());
    }
}



int main(int argc,char**argv)
{
	ros::init(argc, argv, "pololu_servo", ros::init_options::AnonymousName);
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

    // Load parameters
	ROS_INFO("Loading parameters");

	if (nh.hasParam("pololu_motors_yaml"))
    {
        nh.getParam("pololu_motors_yaml", pololu_config_dir);
        YAML::Node motors_config = YAML::LoadFile(pololu_config_dir);

        try
        {
            for(YAML::const_iterator it=motors_config.begin(); it!=motors_config.end(); ++it)
            {
                Motor motor = it->second.as<Motor>();
                motor.name = it->first.as<string>();
                motors[motor.name] = motor;

                // Calculate direction
                int min_sgn = sgn(motor.min);
                int max_sgn = sgn(motor.max);

                // Check min / max signs
                if(min_sgn == max_sgn || max_sgn < 0 || 0 < min_sgn)
                {
                    ROS_ERROR("motor %s, min (%f) and max (%f) cannot have the same sign, max cannot be less than 0 and min cannot be greater than 0", motor.name.c_str(), rad_to_deg(motor.min), rad_to_deg(motor.max));
                    return EXIT_FAILURE;
                }

                //Check init inbetween min and max
//                if(motor.init < motor.min || motor.max < motor.init)
//                {
//                    ROS_ERROR("motor %s, init (%f) out of range of min (%f) and max (%f)", motor.name.c_str(), motor.init, motor.min, motor.max);
//                    return EXIT_FAILURE;
//                }

                //
                double min_range = fabs(-M_PI/2 - motor.init);
                double max_range = fabs(M_PI/2 - motor.init);

                if(fabs(motor.min) > min_range)
                {
                    ROS_ERROR("motor %s, -ve range (%f) cannot be less than %f", motor.name.c_str(), rad_to_deg(motor.min), rad_to_deg(-min_range));
                    return EXIT_FAILURE;
                }
                else if(motor.max > max_range)
                {
                    ROS_ERROR("motor %s, +ve range (%f) cannot be greater than %f", motor.name.c_str(), rad_to_deg(motor.max), rad_to_deg(max_range));
                    return EXIT_FAILURE;
                }

                ROS_INFO("Added motor (id: %d, name: %s, min: %f, init: %f, max: %f)", motor.id, motor.name.c_str(), rad_to_deg(motor.min), rad_to_deg(motor.init), rad_to_deg(motor.max));
            }
        }
        catch(YAML::ParserException& e)
        {
            ROS_ERROR("Problem parsing pololu_motors_yaml: %s", e.what());
            return EXIT_FAILURE;
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
    nh.param<int>("rate_hz", rate_hz, 10);
    pwm_value_range = pwm_max_value - pwm_min_value;

    // Create serial interface
	ROS_INFO("Creating serial interface '%s' at %d bauds\n", port_name.c_str(), baud_rate);
	serial_interface = Polstro::SerialInterface::createSerialInterface(port_name, baud_rate);
	if (!serial_interface->isOpen())
	{
		ROS_ERROR("Failed to open interface, exiting");
		return EXIT_FAILURE;
	}

    // Setup publisher and subscriber
	pub = n.advertise<MotorStateList>(pololu_namespace + "/motor_states", 10);
	ros::Subscriber sub = n.subscribe(pololu_namespace + "/command", 20, motor_command_callback);

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