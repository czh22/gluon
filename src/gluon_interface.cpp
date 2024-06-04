#include "../include/gluon_interface.h"


Gluon_Interface::Gluon_Interface()
{
    sleep(5);
    //Initialize the controller
    pController = ActuatorController::initController();
    //ec Define an error type, ec==0x00 means no error, ec will be passed to pcontroller-> lookupActuators(ec) by reference,
    //when the error occurs, ec value will be modified by SDK to the corresponding error code
    Actuator::ErrorsDefine ec;
    //Find the connected actuators and return the UnifiedID of all actuators found.
    std::vector<ActuatorController::UnifiedID> uIDArray = pController->lookupActuators(ec);
    //If the size of the uIDArray is greater than zero, the connected actuators have been found
    if (uIDArray.size() > 0)
    {
        for(int k = 0; k < uIDArray.size(); k++) {
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
    }
    left_joint_state.name.resize(6);
    left_joint_state.position.resize(6);
    left_joint_state.name[0]="left_axis_joint_1";
    left_joint_state.name[1]="left_axis_joint_2";
    left_joint_state.name[2]="left_axis_joint_3";
    left_joint_state.name[3]="left_axis_joint_4";
    left_joint_state.name[4]="left_axis_joint_5";
    left_joint_state.name[5]="left_axis_joint_6";
    right_joint_state.name.resize(6);
    right_joint_state.position.resize(6);
    right_joint_state.name[0]="right_axis_joint_1";
    right_joint_state.name[1]="right_axis_joint_2";
    right_joint_state.name[2]="right_axis_joint_3";
    right_joint_state.name[3]="right_axis_joint_4";
    right_joint_state.name[4]="right_axis_joint_5";
    right_joint_state.name[5]="right_axis_joint_6";
    std::thread thread_1(&Gluon_Interface::update_state, this);
    thread_1.detach();
}


void Gluon_Interface::update_state()
{
    Rate rate(50);
    while(1)
    {
        for(int i = 0; i < 6; i++) 
        {
            
            left_joint_state.position[i] = POS_TO_RAD(pController->getPosition(uint8_t(i+1),true));

        }

        for(int i = 0; i < 6; i++) 
        {
    
            right_joint_state.position[i] = POS_TO_RAD(pController->getPosition(uint8_t(i+7),true));

        }
        rate.sleep();
    }
}