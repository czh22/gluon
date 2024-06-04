#ifndef GLUON_INTERFACE_H_
#define GLUON_INTERFACE_H_

#include "../ActuatorController_SDK/sdk/include/actuatorcontroller.h"
#include <iostream>
#include <thread>
#include <ros/console.h>
#include <ros/ros.h>
#include "rate.h"
#include <sensor_msgs/JointState.h>


#define DEG_TO_RAD(x) ((x)*M_PI / 180.0)
#define RAD_TO_DEG(x) ((x)*180.0 / M_PI)

#define STEERING_GEAR_RATIO 36

#define RAD_TO_POS(x) ((x / (2 * M_PI)) * STEERING_GEAR_RATIO)
#define POS_TO_RAD(x) ((x * (2 * M_PI)) / STEERING_GEAR_RATIO)


class Gluon_Interface
{
    public:
        Gluon_Interface();
        ~Gluon_Interface() = default;
        ActuatorController * pController;
        sensor_msgs::JointState right_joint_state;
        sensor_msgs::JointState left_joint_state;
        

    private:
        void update_state();
};


#endif
