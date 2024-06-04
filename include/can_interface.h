#ifndef CAN_INTERFACE_H_
#define CAN_INTERFACE_H_

#include <linux/can.h>
#include <sys/socket.h>
#include "ros/ros.h"
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <stdlib.h>
#include <net/if.h>
#include "rate.h"
#include <thread>

#define MAX 10000//电流最大值

#define SUMMAX 1000//I控制器积分最大值，SUMMAX*ki=I控制器电流最大值
#define DMAX 5000//D控制器电流最大值
#define PMAX 6000

class Can_Interface
{
    public:
        Can_Interface();
        ~Can_Interface() = default;
        struct state_6020//6020反馈状态
        {
            uint16_t ecd;
            int16_t speed_rpm;
            int16_t give_current;
            uint8_t temperate;
            int16_t last_ecd;
            int circle;
            double angle;
        };
        struct pid//pid控制相关
        {
            double error;
            double kp;
            double ki;
            double sum_error;
            double kd;
            double last_error;


        };
        int s;//套接字
        can_filter filter;
        can_frame* tx_frame=(can_frame*) malloc(sizeof(can_frame));
        can_frame* rx_frame=(can_frame*) malloc(sizeof(can_frame));
        state_6020* state=(state_6020*) malloc(sizeof(state_6020));
        pid* ppid=(pid*) malloc(sizeof(pid));
        
        int current;
        
        void give_current(uint16_t current);
        void read_states(can_frame* rx_frame,state_6020* state);
        int caculate_pid(double set_angle);
      


    private:
        void update_state();

};


#endif
