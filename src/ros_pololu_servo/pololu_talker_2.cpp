#include <ros_pololu_servo/PololuController.h>
#include <ros_pololu_servo/PololuMath.h>
#include <string>
#include <ros/ros.h>
#include <ros_pololu_servo/MotorCommand.h>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
# define M_PI           3.14159265358979323846
/**
 * Código para a publicação de mensagens do tipo ros_pololu_servo::MotorCommand no tópico /pololu/comand
 */
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
    ros::init(argc, argv, "talker2");

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
    ros::Publisher pub = n.advertise<ros_pololu_servo::MotorCommand>("/pololu/command", 1000); //avisa que irá publicar no tópico /pololu/command

    ros::Rate loop_rate(10);


    int mot_pos = 0;      //variável para determinar para qual motor a mensagem será enviada
    int position=0;       //variável que determina posição do motor que será enviada na mensagem
    int position_aux=0;
    int mode=1;           //modo de operação do nó talker2:: 0= variable 1 = manual.
    int flag_inic=0;      //flag para determinar se o nó está no primeiro loop ou não
    int flag_posit=0;     //flag para determinar se a opsição aumentará ou diminuirá no modo automático. 0 -> posit increasing, 1-> posit decreasing
    int taxa=1;           //variável para a taxa de variação da posição no modo automático
    char c;


    ros_pololu_servo::MotorCommand mtr;     //objeto da mensagem que será publicada
    while (ros::ok())
    {
        if(mode==1)
            do
            {
                ROS_INFO("\nSelect Mode:\n(0)- automatically variable (with one motor)\n(1)- manual\n(2)- automatically variable with five motors (not recommended)");
            }
            while (((scanf("%d%c", &mode, &c)!=2 || c!='\n') && clean_stdin()));
        if(mode!=0 && mode != 2 && mode !=1)
        {
            ROS_INFO("Mode selected is invalid, setting to manual mode");
            mode =1;
        }
        if(mode==1)
            do
            {
                ROS_INFO("Enter position. ");
            }
            while (((scanf("%d%c", &position, &c)!=2 || c!='\n') && clean_stdin()));
        if(mode==1||(mode==0&&flag_inic==0))
            do
            {
                ROS_INFO("Which motor?");
            }
            while (((scanf("%d%c", &mot_pos, &c)!=2 || c!='\n') && clean_stdin()));


        if((mode==0||mode==2)&&flag_inic==0){
            flag_inic=1;
            do
            {
                ROS_INFO("Enter initial position, please");
            }
            while (((scanf("%d%c", &position, &c)!=2 || c!='\n') && clean_stdin())); 
            do
            {
                ROS_INFO("Enter velocity, please");
            }
            while (((scanf("%d%c", &taxa, &c)!=2 || c!='\n') && clean_stdin())); 
            if(taxa<0){
                taxa=-taxa;
                flag_posit=!flag_posit;
            }
        }
        if(mode==0)
        {
            if(!flag_posit)
                position+=taxa;
            else
                position-=taxa;
            if(position>=45)
            {
                for(; position<1000000; position++); //for dummie para ficar um tempo parado na posição máxima
                flag_posit=1;
                position =45;
            }
            else if(position<=-45)
            {
                for(; position<1000000; position++); //for dummie para ficar um tempo parado na posição máxima
                flag_posit=0;
                position =-45;
            }
        }
        if(mode==2)
        {
            /**aqui serão utilizados os motores 0 (propulsão) 6 e 8 (vetorização) 12 e 14 (lemes)*/
            if(!flag_posit)
                position_aux+=taxa;
            else
                position_aux-=taxa;
            if(position_aux>=100)
            {
                flag_posit=1;
                position_aux =100;
            }
            else if(position_aux<=-100)
            {
                flag_posit=0;
                position_aux =-100;
            }
            if(mot_pos==0)
            {
                mot_pos=6;
                position=(int)position_aux*0.45;
            }
            else if(mot_pos==6)
            {
                mot_pos=8;
                position=(int)position_aux*0.3;
            }
            else if(mot_pos==8)
            {
                mot_pos=12;
                position=(int)position_aux*(-0.3);
            }
            else if(mot_pos==12)
            {
                mot_pos=14;
                position=(int)position_aux*0.45;
            }
            else if(mot_pos==14)
            {
                mot_pos=0;
                position=(int)position_aux*(-0.45);
            }
//ROS_INFO("\rmotor escolhido: %d",mot_pos);
        }
        if (mot_pos == 0)
            mtr.joint_name = "prop_one";
        else if (mot_pos == 1)
            mtr.joint_name = "prop_two";
        else if (mot_pos == 2)
            mtr.joint_name = "prop_three";
        else if (mot_pos == 3)
            mtr.joint_name = "prop_four";
        else if (mot_pos == 6)
            mtr.joint_name = "vet_one";
        else if (mot_pos == 7)
            mtr.joint_name = "vet_two";
        else if (mot_pos == 8)
            mtr.joint_name = "vet_three";
        else if (mot_pos == 9)
            mtr.joint_name = "vet_four";
        else if (mot_pos == 12)
            mtr.joint_name = "leme_one";
        else if (mot_pos == 13)
            mtr.joint_name = "leme_two";
        else if (mot_pos == 14)
            mtr.joint_name = "leme_three";
        else if (mot_pos == 15)
            mtr.joint_name = "leme_four";
        else if (mot_pos == 22)
            mtr.joint_name = "digital_one";
        else if (mot_pos == 23)
            mtr.joint_name = "digital_two";
        else
        {
            mtr.joint_name = "prop_one";
            ROS_INFO("invalid number, setting motor to prop_one");
        }
        mtr.position = int(position)*M_PI/180;
        mtr.speed = 1.0;
        mtr.acceleration=1.0;
        /**
        * The publish() function is how you send messages. The parameter
        * is the message object. The type of this object must agree with the type
        * given as a template parameter to the advertise<>() call, as was done
        * in the constructor above.
        */
        pub.publish(mtr);
        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}

