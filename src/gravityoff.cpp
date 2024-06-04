#include <iostream>
#include "../ActuatorController_SDK/sdk/include/actuatorcontroller.h"
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

#include "kinematics_dynamics.h"
#include "teaching.h"

#define DEG_TO_RAD(x) ((x)*M_PI / 180.0)
#define RAD_TO_DEG(x) ((x)*180.0 / M_PI)

#define STEERING_GEAR_RATIO 36

#define RAD_TO_POS(x) ((x / (2 * M_PI)) * STEERING_GEAR_RATIO)
#define POS_TO_RAD(x) ((x * (2 * M_PI)) / STEERING_GEAR_RATIO)

using namespace std;


// 是否停止录制
int stopRecording = 0;
float frequency = 50;

void ifStoping()
{
  while(1)
  {
    std::cout<<"type 'yes' to stop recording."<<std::endl;
    std::string answer;
    std::cin>>answer;
    if(answer == "yes")
    {
      stopRecording = 1;
    }
  }
}
// 录制数据文件名称
std::string left_file_name = "left";
std::string right_file_name = "right";
// 录制数据文件保存路径
std::string save_path = "/home/s/Desktop/catkin_workspace/src/gluon/record";
// string urdf_path = "/home/s/Desktop/catkin_workspace/src/gluon/urdf/gravity.urdf";
KinematicsDynamics left_kinematic_dynamics("/home/s/Desktop/catkin_workspace/src/gluon/urdf/left_gravity.urdf");
KinematicsDynamics right_kinematic_dynamics("/home/s/Desktop/catkin_workspace/src/gluon/urdf/right_gravity.urdf");

double joint_state[12];
//vector<double> left_joint_states = {0.0,0.0,0.0,0.0,0.0,0.0};
vector<double> left_id_results = {0.0,0.0,0.0,0.0,0.0,0.0};
vector<double> left_id_mutiple = {1.3,0.0,0.0,0.0,0.0,0.0};
vector<double> right_id_results = {0.0,0.0,0.0,0.0,0.0,0.0};
vector<double> right_id_mutiple = {1.3,0.0,0.0,0.0,0.0,0.0};
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gravityoff");
  ros::NodeHandle nh_;

  ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states",10);
  sensor_msgs::JointState left_joint_state;
  
  left_joint_state.name.resize(6);
  left_joint_state.position.resize(6);
  left_joint_state.name[0]="left_axis_joint_1";
  left_joint_state.name[1]="left_axis_joint_2";
  left_joint_state.name[2]="left_axis_joint_3";
  left_joint_state.name[3]="left_axis_joint_4";
  left_joint_state.name[4]="left_axis_joint_5";
  left_joint_state.name[5]="left_axis_joint_6";

  sensor_msgs::JointState right_joint_state;
  
  right_joint_state.name.resize(6);
  right_joint_state.position.resize(6);
  right_joint_state.name[0]="right_axis_joint_1";
  right_joint_state.name[1]="right_axis_joint_2";
  right_joint_state.name[2]="right_axis_joint_3";
  right_joint_state.name[3]="right_axis_joint_4";
  right_joint_state.name[4]="right_axis_joint_5";
  right_joint_state.name[5]="right_axis_joint_6";


    
  //Initialize the controller
  ActuatorController * pController = ActuatorController::initController();
  //ec Define an error type, ec==0x00 means no error, ec will be passed to pcontroller-> lookupActuators(ec) by reference,
  //when the error occurs, ec value will be modified by SDK to the corresponding error code
  Actuator::ErrorsDefine ec;
  //Find the connected actuators and return the UnifiedID of all actuators found.
  std::vector<ActuatorController::UnifiedID> uIDArray = pController->lookupActuators(ec);
  //If the size of the uIDArray is greater than zero, the connected actuators have been found
  sleep(5);
  if (uIDArray.size() > 0)
  {
    for(int k = 0; k < uIDArray.size(); k++) 
    {
      ActuatorController::UnifiedID actuator = uIDArray.at(k);
      //Enable actuator
      ROS_INFO("actuator ID %d, ipAddr %s", actuator.actuatorID,actuator.ipAddress.c_str());
      pController->enableActuator(actuator.actuatorID,actuator.ipAddress);
      //activate profile position mode
      pController->activateActuatorMode(actuator.actuatorID,Actuator::Mode_Profile_Pos);

      cout << "set position to 10 revolutions " << endl;
      pController->setPosition(actuator.actuatorID,0);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      
      
    }
        //Disable all connected actuators
        //pController->disableAllActuators();
        //insure that all actuators have been closed
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  else
  {
      cout << "Connected error code:" << hex << ec << endl;
      return -1;
  }
  Recorder left_recorder(save_path, left_file_name, frequency);
  Recorder right_recorder(save_path, right_file_name, frequency);
  for(int i = 0;i<12;i++)
  {
    pController->activateActuatorMode(uint8_t(i+1),Actuator::Mode_Cur);
    pController->setCurrent(uint8_t(i+1),0.0);
  }

  // 创建线程
  std::thread thread_1(ifStoping);
  thread_1.detach();
  ros::Rate loop_rate(frequency);
  while(!stopRecording) 
  {
      ros::spinOnce();
      loop_rate.sleep();
      
      for(int i = 0; i < 6; i++) 
      {
        left_joint_state.position[i] = POS_TO_RAD(pController->getPosition(uint8_t(i+1),true));
        left_recorder.writeData(left_joint_state.position);
      }
      for(int i = 0; i < 6; i++) 
      {
        right_joint_state.position[i] = POS_TO_RAD(pController->getPosition(uint8_t(i+7),true));
        right_recorder.writeData(right_joint_state.position);
      }
     

      left_kinematic_dynamics.updateJointStates(left_joint_state.position);
      left_id_results = left_kinematic_dynamics.solveID({0.0,0.0,0.0,0.0,0.0,0.0});
      right_kinematic_dynamics.updateJointStates(right_joint_state.position);
      right_id_results = right_kinematic_dynamics.solveID({0.0,0.0,0.0,0.0,0.0,0.0});
      // cout<<"left ID result: "<<left_id_results[0]<<", "<<left_id_results[1]<<", "<<left_id_results[2]<<", "<<left_id_results[3]<<", "<<left_id_results[4]<<", "<<left_id_results[5]<<endl;
      // cout<<"right ID result: "<<right_id_results[0]<<", "<<right_id_results[1]<<", "<<right_id_results[2]<<", "<<right_id_results[3]<<", "<<right_id_results[4]<<", "<<right_id_results[5]<<endl;
    
      for(int i = 0;i<6;i++)
      {
        pController->setCurrent(uint8_t(i+1),left_id_mutiple[i]*left_id_results[i]);
      }
      for(int i = 0;i<6;i++)
      {
        pController->setCurrent(uint8_t(i+7),right_id_mutiple[i]*right_id_results[i]);
      }
      left_joint_state.header.stamp = ros::Time::now();
      right_joint_state.header.stamp = ros::Time::now();
      
      joint_pub.publish(left_joint_state);
      joint_pub.publish(right_joint_state);
  }
  left_recorder.endRecording();
  right_recorder.endRecording();


  return 0;
}
