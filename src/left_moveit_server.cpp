//action服务端的相关定义，请加入到驱动节点的头文件中
#include "actionlib/server/action_server.h"    

//action服务端的目标控制句柄定义，与接收的目标相关联后，可以用来实现action的信息反馈等操作       
#include "actionlib/server/server_goal_handle.h" 

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <sensor_msgs/JointState.h>

#include <iostream>
#include <thread>

#define DEG_TO_RAD(x) ((x)*M_PI / 180.0)
#define RAD_TO_DEG(x) ((x)*180.0 / M_PI)

#define STEERING_GEAR_RATIO 36

#define RAD_TO_POS(x) ((x / (2 * M_PI)) * STEERING_GEAR_RATIO)
#define POS_TO_RAD(x) ((x * (2 * M_PI)) / STEERING_GEAR_RATIO)

int n_tra_Points;
int n_joints;
ros::Publisher left_joint_pub;
sensor_msgs::JointState left_joint_state;


class Gluon_driver
{
    protected:
    ros::NodeHandle nh_;
    //定义action服务端
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>  as_;    

    //定义action服务端目标控制句柄
    actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle_; 

    //用来反馈action目标的执行情况，客户端由此可以得知服务端是否执行成功了
    control_msgs::FollowJointTrajectoryResult result_;   
    moveit_msgs::RobotTrajectory moveit_tra;
    
    
    public:
    

    
    
    Gluon_driver():as_(nh_, "left_gluon/left_follow_joint_trajectory",boost::bind(&Gluon_driver::goal_callback, this, _1),false)
    {
           
        left_joint_pub = nh_.advertise<sensor_msgs::JointState>("left_joint_states",10);
        
        left_joint_state.name.resize(6);
        left_joint_state.position.resize(6);
        left_joint_state.name[0]="left_axis_joint_1";
        left_joint_state.name[1]="left_axis_joint_2";
        left_joint_state.name[2]="left_axis_joint_3";
        left_joint_state.name[3]="left_axis_joint_4";
        left_joint_state.name[4]="left_axis_joint_5";
        left_joint_state.name[5]="left_axis_joint_6";
        
        as_.start();
        
    };

    



    void goal_callback( actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
    {
        actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal = *gh.getGoal();
        goal_handle_ = gh;
        moveit_tra.joint_trajectory.header.frame_id = goal.trajectory.header.frame_id;
        moveit_tra.joint_trajectory.joint_names = goal.trajectory.joint_names;
        std::cout<<"left frame_id: "<<moveit_tra.joint_trajectory.header.frame_id<<std::endl;
        n_joints = goal.trajectory.joint_names.size();
        std::cout<<"robot has "<<n_joints<<" left joints   "<<goal.trajectory.joint_names[0]<<goal.trajectory.joint_names[1]<<goal.trajectory.joint_names[2]<<goal.trajectory.joint_names[3]<<goal.trajectory.joint_names[4]<<goal.trajectory.joint_names[5]<<std::endl;
        n_tra_Points = goal.trajectory.points.size();
        std::cout<<"left traj has "<<n_tra_Points<<" points"<<std::endl;
        moveit_tra.joint_trajectory.points.resize(n_tra_Points);
        for(int i=0; i<n_tra_Points; i++) // 遍历每组路点
        {
            moveit_tra.joint_trajectory.points[i].positions.resize(n_joints);
            moveit_tra.joint_trajectory.points[i].velocities.resize(n_joints);
            moveit_tra.joint_trajectory.points[i].accelerations.resize(n_joints);

            moveit_tra.joint_trajectory.points[i].time_from_start = goal.trajectory.points[i].time_from_start;
            for(int j=0;j<n_joints; j++) // 遍历每组路点中的每个关节数据
            {
                moveit_tra.joint_trajectory.points[i].positions[j] = goal.trajectory.points[i].positions[j];
                
                moveit_tra.joint_trajectory.points[i].velocities[j] = goal.trajectory.points[i].velocities[j];
                moveit_tra.joint_trajectory.points[i].accelerations[j] = goal.trajectory.points[i].accelerations[j];
            }
        }
        excute_traj();

        goal_handle_.setAccepted();    
        result_.error_code = result_.SUCCESSFUL;

        goal_handle_.setSucceeded(result_);

    };

    void excute_traj()
    {
        for(int i=0; i<n_tra_Points; i++) // 遍历每组路点
        {
            
            for(int j=0;j<n_joints; j++) // 遍历每组路点中的每个关节数据
            {
                    left_joint_state.position[j] = moveit_tra.joint_trajectory.points[i].positions[j];
                    
            }
            left_joint_state.header.stamp = ros::Time::now();
            left_joint_pub.publish(left_joint_state);
            usleep(100000);
        }
    };

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "left_moveit_server");
    
    Gluon_driver gluon_driver;
    
    ros::spin();
    return 0;
}