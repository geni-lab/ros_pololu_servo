#include <ros/ros.h>
#include <ros_pololu_servo/DigitalCommand.h>
#include <ros_pololu_servo/DigitalState.h>
#include <stdio.h>
#include <stdlib.h>
/**
 * Código para a publicação de mensagens do tipo ros_pololu_servo::MotorCommand no tópico /pololu/comand 
 * caso o nó controller não esteja no modo de leitura contínua
 */

void digital_received_callback(const ros_pololu_servo::DigitalState::ConstPtr& msg);
int clean_stdin()
{
    while (getchar()!='\n');
    return 1;
}

int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line. For programmatic
     * remappings you can use a different version of init() which takes remappings
     * directly, but for most command-line programs, passing argc and argv is the easiest
     * way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "reader");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher pub = n.advertise<ros_pololu_servo::DigitalCommand>("/pololu/command_digital", 100); 
    //avisa que irá publicar no tópico /pololu/command_digital

    ros::Rate loop_rate(100);


    bool command = false; 
    int mode=1;
    char c;


    ros_pololu_servo::DigitalCommand dig_msg;     //objeto da mensagem que será publicada
    while (ros::ok())
    {
        if(mode==0) return 0;
        if(mode==1)
            do
            {
                ROS_INFO("\nPress 1 if you want to read the state of the digital inputs, to kill the node press 0.\n");
            }
            while (((scanf("%d%c", &mode, &c)!=2 || c!='\n') && clean_stdin()));

        if(mode==1)
            command = true;

        if(mode!=0 && mode != 1)
        {
            ROS_INFO("Mode selected is invalid");
            mode =1;
            command = false;
        }

        if(mode==0)
        {
            ROS_INFO("Shutting down this node.\n");
            command= false;
        }

        
        
        /**
        * The publish() function is how you send messages. The parameter
        * is the message object. The type of this object must agree with the type
        * given as a template parameter to the advertise<>() call, as was done
        * in the constructor above.
        */
        dig_msg.command = command;
        pub.publish(dig_msg);
        loop_rate.sleep();
        ros::spinOnce();
 
    }



    return 0;
}

