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
#include <gluon_interface.h>
#include<teaching.h>





using namespace std;
// 录制数据文件名称
std::string left_file_name = "left";
std::string right_file_name = "right";
// 录制数据文件保存路径
std::string save_path = "/home/s/Desktop/catkin_workspace/src/gluon/record";



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dual_replay");
    ros::NodeHandle nh_;
    Gluon_Interface gluon_interface;
    Player left_player(save_path, left_file_name);
    Player right_player(save_path, right_file_name);
    float frequency = left_player.getFrequency();
    ros::Rate loop_rate(frequency);
    // 关节状态（角度）
    std::vector<double> left_joint_states = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> right_joint_states = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    while(ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
        left_joint_states = left_player.getData();
        right_joint_states = right_player.getData();
        for(int i = 0;i<6;i++)
        {
            gluon_interface.pController->setPosition(uint8_t(i+1),STEERING_GEAR_RATIO/2/M_PI*left_joint_states[i]);
        }
        for(int i = 0;i<6;i++)
        {
            gluon_interface.pController->setPosition(uint8_t(i+7),STEERING_GEAR_RATIO/2/M_PI*right_joint_states[i]);
        }
    }
    return 0;
}