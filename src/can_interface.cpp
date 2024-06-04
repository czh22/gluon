#include "../include/can_interface.h"





Can_Interface::Can_Interface()
{
    system("sudo ifconfig can0 down");
    system("sudo ip link set can0 type can bitrate 1000000");
    system("sudo ifconfig can0 up");
    
    tx_frame->can_dlc=8;
    tx_frame->can_id=0x1ff;
    filter.can_id = 0x205;//6020电机id为1，标识符=204+id
    filter.can_mask = CAN_SFF_MASK;
    struct sockaddr_can addr;
    struct ifreq ifr;
    s = socket(PF_CAN,SOCK_RAW,CAN_RAW);
    strcpy(ifr.ifr_name,"can0");
    ioctl(s,SIOCGIFINDEX,&ifr);
    addr.can_family=AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s,(struct sockaddr *)&addr,sizeof(addr));
    setsockopt(s,SOL_CAN_RAW,CAN_RAW_FILTER,&filter,sizeof(filter));
    ppid->error=0;
    ppid->kp=350;
    ppid->ki=0;
    ppid->kd=8000;
    ppid->sum_error=0;
    ppid->last_error=0;
    read_states(rx_frame,state);
    //初始化时根据电机码盘值确定实际位置
    if(state->ecd<3000&&state->ecd>0)
    {
        state->circle=1;
    }
    else if(state->ecd>3000&&state->ecd<8192)
    {
        state->circle=0;
    }
    std::thread thread_1(&Can_Interface::update_state, this);
    thread_1.detach();
}


void Can_Interface::give_current(uint16_t current)
{
    std::fill(std::begin(tx_frame->data),std::end(tx_frame->data),0);
        
    tx_frame->data[0] = static_cast<uint8_t>(current>>8u);
    tx_frame->data[1] = static_cast<uint8_t>(current);
    write(s,tx_frame,sizeof(*tx_frame));

}


void Can_Interface::read_states(can_frame* rx_frame,state_6020* state)
{
    int nbytes = read(s,rx_frame,sizeof(*rx_frame));
        if(nbytes > 0)
        {
            //解码反馈信息
            state->last_ecd = state->ecd;
            state->ecd = (uint16_t)((rx_frame->data[0])<<8|(rx_frame->data[1]));
            state->speed_rpm = (uint16_t)((rx_frame->data[2])<<8|(rx_frame->data[3]));
            state->give_current = (uint16_t)((rx_frame->data[4])<<8|(rx_frame->data[5]));
            state->temperate = rx_frame->data[6];

            //printf("ID=0x%x DLC=%d encoder=%d\n" ,rx_frame->can_id,rx_frame->can_dlc,state->ecd);
        }

}

int Can_Interface::caculate_pid(double set_angle)
{
    ppid->error=set_angle-state->angle;
    double dout=ppid->kd*(ppid->error-ppid->last_error);
    if(dout>DMAX)
    {
        dout=DMAX;
    }
    if(dout<-DMAX)
    {
        dout=-DMAX;
    }
    printf("dout=%lf\n",dout);
    
    
    ppid->last_error=ppid->error;
    ppid->sum_error+=ppid->error;
    
    if(ppid->sum_error>SUMMAX)
    {
        ppid->sum_error=SUMMAX;
    }
    if(ppid->sum_error<-SUMMAX)
    {
        ppid->sum_error=-SUMMAX;
    }
    double pout=ppid->kp*ppid->error;
    if(pout>PMAX)
    {
        pout=PMAX;
    }
    if(pout<-PMAX)
    {
        pout=-PMAX;
    }
    double iout=ppid->ki*ppid->sum_error;
    
    //printf("sum=%lf\n",ppid->sum_error);
    double current=pout+iout+dout;
    if(current>MAX)
    {
        current=MAX;
    }
    else if(current<-MAX)
    {
        current=-MAX;
    }
    //printf("c=%lf\n",current);
    return current;


}

void Can_Interface::update_state()
{
    Rate rate(1000);
    while(1)
    {
        read_states(rx_frame,state);
        //过零检测
        if(state->ecd-state->last_ecd > 4000)
        {
            state->circle--;
        }
        else if(state->ecd-state->last_ecd < -4000)
        {
            state->circle++;
        }
        state->angle = ((state->ecd+state->circle*8192-1500)/12288.0)*180;
        //printf("angle=%lf\n",state->angle);
        rate.sleep();
    }
    
}