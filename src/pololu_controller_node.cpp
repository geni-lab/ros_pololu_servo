#include "PololuController.h"

int main(int argc,char**argv)
{
    ros::init(argc, argv, "pololu_servo");
    PololuController controller;

    if(!controller.initialize())
    {
        ROS_INFO("Failed to initialize ros_polou_controller_node");
        return EXIT_FAILURE;
    }
    else
    {
        ROS_INFO("Successfully initialized ros_polou_controller_node");
    }

    ros::Rate rate(controller.get_rate_hz());

	while(ros::ok())
	{
	    controller.publish_motor_state();
		ros::spinOnce();
        rate.sleep();
	}

	//controller.~PololuController();

	ROS_INFO("Exited ros_polou_controller_node");

	return EXIT_SUCCESS;
}