#include <ros/ros.h>
#include <vector>
#include <ros_pololu_servo/servo_pololu.h>
//#include <ros_pololu_servo/pololu_state.h>
#include <ros_pololu_servo/motor.h>
#include <ros_pololu_servo/pololu_info.h>
#include "PolstroSerialInterface.h"

#include "yaml-cpp/yaml.h"

const unsigned int baudRate = 115200;//38400
const float pi = 3.141592653589793f;
const unsigned int channelMinValue = 4000;
const unsigned int channelMaxValue = 8000;
const unsigned int channelValueRange = channelMaxValue - channelMinValue;
const unsigned int signalPeriodInMs = 2000;
Polstro::SerialInterface* serialInterface;
std::string portName = "/dev/ttyACM0";
ros_pololu_servo::servo_pololu msgTemp,msgs;
ros_pololu_servo::motor motor;
ros_pololu_servo::pololu_info inf;
YAML::Node config = YAML::LoadFile("/rsm/pololu.yaml");
ros::Publisher pub;
/*
bool status(ros_pololu_servo::pololu_state::Request  &req, 
ros_pololu_servo::pololu_state::Response &res)
{
	//
	unsigned char channelNumber=req.qid;
	bool ret;
	unsigned short position;
	res.id;
	serialInterface->getPositionCP( channelNumber, position );
	res.angle=(((float)(position-channelMinValue)/(float)channelValueRange)-0.5)*pi;
	ROS_INFO("getPositionCP(%d) (ret=%d position=%d)\n", channelNumber, ret, position );
	return true;
}
*/
void pub_motors()
{
	inf.info.clear();
	for (std::size_t i=0; i<config.size(); i++) 
	{
		//std::cout << config[i]["id"].as<std::string>() << "\n";
	unsigned char channelNumber=config[i]["id"].as<unsigned char>();
	unsigned short position;
	serialInterface->getPositionCP( channelNumber, position );
	float angle=(((float)(position-channelMinValue)/(float)channelValueRange)-0.5)*pi;
	ROS_INFO("getPositionCP(%d) (position=%d)\n", channelNumber, position );
	motor.id=channelNumber;
	motor.name=config[i]["name"].as<std::string>();
	motor.position=angle;
	inf.info.push_back(motor);
	}
	pub.publish(inf);
}
/*
void timerCallback(const ros::TimerEvent& e)
{
	pub_motors();
}
*/
void CommandCallback(const ros_pololu_servo::servo_pololu::ConstPtr& msg)
{
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	//msgTemp=(*msg);
	unsigned char channelNumber=msg->id;
	if (msg->speed>0)serialInterface->setSpeedCP( channelNumber, msg->speed );
	if (msg->acceleration>0)serialInterface->setAccelerationCP( channelNumber, msg->acceleration );
	if ((msg->angle>=-pi/2)&&(msg->angle<=pi/2)){
		//
		int i=((msg->angle+pi/2.0)/pi)*(float)channelValueRange+(float)channelMinValue;
		ROS_INFO("motor %d : position=%d",channelNumber,i);
		if (i>=channelMinValue && i<=channelMaxValue)
		serialInterface->setTargetCP( channelNumber, i );
	}
}


int main(int argc,char**argv)
{
	ros::init(argc, argv, "pololu_servo");
	ros::NodeHandle n;
	ROS_INFO("Creating serial interface '%s' at %d bauds\n", portName.c_str(), baudRate);
	serialInterface = Polstro::SerialInterface::createSerialInterface( portName, baudRate );
	if ( !serialInterface->isOpen() )
	{
		ROS_ERROR("Failed to open interface\n");
		return -1;
	}
	
	pub = n.advertise<ros_pololu_servo::pololu_info>("pololu_info", 100);
	////ros::Timer timer = n.createTimer(ros::Duration(0.5), timerCallback);
	ros::Subscriber sub = n.subscribe("/cmd_pololu", 20, CommandCallback);
	//ros::ServiceServer service = n.advertiseService("pololu_status", status);
    ROS_INFO("Ready...");
	//
	////ros::spin();
	
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ros::spinOnce();
        loop_rate.sleep();
        pub_motors();
	}
	
	ROS_INFO("Deleting serial interface...");
	delete serialInterface;
	serialInterface = NULL;
	return 0;
}
