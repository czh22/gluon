/*
*执行器位置控制
*/

//跑之前先sudo ifconfig can0 down
//sudo ip link set can0 type can bitrate 1000000 设置比特率为1000000
//sudo ifconfig can0 up

#include <iostream>

#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <queue>
#include <vector>

#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <linux/can.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <stdlib.h>
#include <net/if.h>
#include <can_interface.h>
#include <gluon_interface.h>




using namespace std;

double joint_state[12];

double set_angle = 90;

void keyboard()
{
    while(1)
    {
        char ch = getchar();
        if(ch == 'q')
            joint_state[0] += 0.1;
        if(ch == 'w')
            joint_state[1] += 0.1;
        if(ch == 'e')
            joint_state[2] += 0.1;
        if(ch == 'r')
            joint_state[3] += 0.1;
        if(ch == 't')
            joint_state[4] += 0.1;
        if(ch == 'y')
            joint_state[5] += 0.1;
        if(ch == 'a')
            joint_state[6] += 0.1;
        if(ch == 's')
            joint_state[7] += 0.1;
        if(ch == 'd')
            joint_state[8] += 0.1;
        if(ch == 'f')
            joint_state[9] += 0.1;
        if(ch == 'g')
            joint_state[10] += 0.1;
        if(ch == 'h')
            joint_state[11] += 0.1;
        if(ch == '1')
            joint_state[0] -= 0.1;
        if(ch == '2')
            joint_state[1] -= 0.1;
        if(ch == '3')
            joint_state[2] -= 0.1;
        if(ch == '4')
            joint_state[3] -= 0.1;
        if(ch == '5')
            joint_state[4] -= 0.1;
        if(ch == '6')
            joint_state[5] -= 0.1;
        if(ch == 'z')
            joint_state[6] -= 0.1;
        if(ch == 'x')
            joint_state[7] -= 0.1;
        if(ch == 'c')
            joint_state[8] -= 0.1;
        if(ch == 'v')
            joint_state[9] -= 0.1;
        if(ch == 'b')
            joint_state[10] -= 0.1;
        if(ch == 'n')
            joint_state[11] -= 0.1;

        if(ch == 'o')
            set_angle = 60;
        if(ch == 'l')
            set_angle = 120;
        
        // for(int i = 0;i<12;i++)
        // {
        //     cout<<"joint "<<i+1<<" = "<<joint_state[i]<<endl;
        // }
        // cout<<"base = "<<set_angle<<endl;
    }

    
}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "gluonControl_withkey");
    ros::NodeHandle nh_;

    system("stty -icanon");
  
    Gluon_Interface gluon_interface;
    int current;
    Can_Interface can_interface;
    thread keyboard_thread(keyboard);
    keyboard_thread.detach();

    ros::Rate loop_rate(50);
    while(ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
      
        
        for(int i = 0; i < 12; i++) 
        {
            gluon_interface.pController->setPosition(uint8_t(i+1), joint_state[i]);
        }
        printf("angle=%lf\n",can_interface.state->angle);
        printf("set_angle=%lf\n",set_angle);
        current = can_interface.caculate_pid(set_angle);//pid计算
        printf("current=%d\n",current);
        can_interface.give_current(current);
    }

    return 0;
}


