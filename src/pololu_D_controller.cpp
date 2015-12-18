#include <ros_pololu_servo/PololuController.h>
#include <ros_pololu_servo/PololuMath.h>
#include <polstro/PolstroSerialInterface.h>
#include <string>
#include <ros/ros.h>
#include <ros_pololu_servo/MotorCommand.h>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <yocto/PWM_info.h>
# define M_PI           3.14159265358979323846
/**
 * Código para simular o controlador, este código irá monitorar as chaves digitais (Comutador, garatéia e válvula de hélio) assim como o sinal recebido pelo RX
 * e com essas informações irá controlar os 12 motores
 */


float position_motors[12]= {0,0,0,0,0,0,0,0,0,0,0,0};           //vetor que determina posição dos motores que serão enviadas nas mensagens
float velocidades[12] ={1, 2,-1,-2,1, 2,-1,-2,1, 2,-1,-2};      //vetor que determina a velocidade em que a posição irá variar
int flag_position[12]= {0,0,0,0,0,0,0,0,0,0,0,0};               //flag para determinar se a opsição aumentará ou diminuirá no modo automático. 0 -> posit increasing, 1-> posit decreasing
int mode=1;                                                     //modo de operação do nó talker2:: 0= variable 1 = manual.
int flag_inic=0;                                                //flag para determinar se o nó está no primeiro loop ou não
float taxa=1;                                                   //variável para a taxa de variação da posição no modo automático
int i;                                                          //variável auxiliar
char c;

float RX_received[4];
bool chaves[3]={0,0,0};

int clean_stdin();
void  digital_received_callback(const ros_pololu_servo::DigitalState::ConstPtr& msg);
void  RX_received_callback(const yocto::PWM_info::ConstPtr& msg);

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
    ros::init(argc, argv, "talker");

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
    ros::Publisher pub_motor = n.advertise<ros_pololu_servo::MotorCommand>("/pololu/command_motor", 1000); //avisa que irá publicar no tópico /pololu/command_motor
    ros::Subscriber sub_digital = n.subscribe("/pololu/digital_state", 100, digital_received_callback);
    ros::Subscriber sub_yocto = n.subscribe("/yocto/pwm_info", 100, RX_received_callback);



    ros::Rate loop_rate(100);


    ros_pololu_servo::MotorCommand mtr;     //objeto da mensagem que será publicada
    while (ros::ok())
    {

    }



    return 0;
}

int clean_stdin()
{
    while (getchar()!='\n');
    return 1;
}

void  digital_received_callback(const ros_pololu_servo::DigitalState::ConstPtr& msg)
{
    chaves[0]=msg->comutador;
    chaves[1]=msg->val_helio;
    chaves[2]=msg->garateia;
}

void  RX_received_callback(const yocto::PWM_info::ConstPtr& msg)
{
    RX_received[0]=msg->duty_cycle_1;
    RX_received[1]=msg->duty_cycle_2;
    RX_received[2]=msg->duty_cycle_3;
    RX_received[3]=msg->duty_cycle_4;

}