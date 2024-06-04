#include <can_interface.h>
#include <sensor_msgs/JointState.h>

#define DEG_TO_RAD(x) ((x)*M_PI / 180.0)
#define RAD_TO_DEG(x) ((x)*180.0 / M_PI)

double set_angle = 270;

void callback(const sensor_msgs::JointStatePtr msg)
{
    set_angle = RAD_TO_DEG(msg->position[0]);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "can_bringup");
    ros::NodeHandle nh_;
    ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states",10);
    ros::Subscriber can_pos =nh_.subscribe("can_state", 10, callback);
    Can_Interface can_interface;
    int current;
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(1);
    joint_state.position.resize(1);
    joint_state.name[0]="cloud_platform_joint";
    ros::Rate loop_rate(50);
    while(ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();

        printf("angle=%lf\n",can_interface.state->angle);
        joint_state.position[0] = DEG_TO_RAD(can_interface.state->angle);
        joint_state.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state);
        printf("set_angle=%lf\n",set_angle);
        current = can_interface.caculate_pid(set_angle);//pid计算
        printf("current=%d\n",current);
        can_interface.give_current(current);
    }
    return 0;
}