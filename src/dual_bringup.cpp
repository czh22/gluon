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





using namespace std;


Gluon_Interface gluon_interface;
ros::Publisher joint_pub;
// ros::Subscriber right_sub_jointstates;
// ros::Subscriber left_sub_jointstates;




void state_publisher()
{
  ros::Rate loop_rate(50);
  while(ros::ok()) 
  {
    ros::spinOnce();
    loop_rate.sleep();      
    
    gluon_interface.left_joint_state.header.stamp = ros::Time::now();
    gluon_interface.right_joint_state.header.stamp = ros::Time::now();
            
    joint_pub.publish(gluon_interface.left_joint_state);
    joint_pub.publish(gluon_interface.right_joint_state);
  }
}


void right_callback(const sensor_msgs::JointStatePtr msg)
{
  for(int i = 0;i<6;i++)
  {
    gluon_interface.pController->setPosition(uint8_t(i+7), STEERING_GEAR_RATIO/2/M_PI*msg->position[i]);
  }
}

void left_callback(const sensor_msgs::JointStatePtr msg)
{
  for(int i = 0;i<6;i++)
  {
    gluon_interface.pController->setPosition(uint8_t(i+1), STEERING_GEAR_RATIO/2/M_PI*msg->position[i]);
  }
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dual_bringup");
  ros::NodeHandle nh_;
  joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states",10);
  ros::Subscriber right_sub_jointstates =nh_.subscribe("/right_joint_states", 10, right_callback);
  ros::Subscriber left_sub_jointstates =nh_.subscribe("/left_joint_states", 10, left_callback);
  thread State_Publisher(state_publisher);
  State_Publisher.detach();
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  return 0;
}
