#include <ros/ros.h>
#include <servo_controller/servo_polulu.h>
#include <servo_controller/polulu_state.h>
#include "PolstroSerialInterface.h"

const unsigned int baudRate = 9600;
const float pi = 3.141592653589793f;
const unsigned int channelMinValue = 4000;
const unsigned int channelMaxValue = 8000;
const unsigned int channelValueRange = channelMaxValue - channelMinValue;
const unsigned int signalPeriodInMs = 2000;
Polstro::SerialInterface* serialInterface;
std::string portName = "/dev/ttyACM0";
servo_controller::servo_polulu msgTemp,msgs;

bool status(servo_controller::polulu_state::Request  &req, 
servo_controller::polulu_state::Response &res)
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


void CommandCallback(const servo_controller::servo_polulu::ConstPtr& msg)
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
	ros::init(argc, argv, "polulu_servo");
	ros::NodeHandle n;
	ROS_INFO("Creating serial interface '%s' at %d bauds\n", portName.c_str(), baudRate);
	serialInterface = Polstro::SerialInterface::createSerialInterface( portName, baudRate );
	if ( !serialInterface->isOpen() )
	{
		ROS_ERROR("Failed to open interface\n");
		return -1;
	}
	ros::Subscriber sub = n.subscribe("/cmd_polulu", 20, CommandCallback);
	ros::ServiceServer service = n.advertiseService("polulu_status", status);
    ROS_INFO("Ready...");
	//
	ros::spin();
	ROS_INFO("Deleting serial interface...");
	delete serialInterface;
	serialInterface = NULL;
	return 0;
}
