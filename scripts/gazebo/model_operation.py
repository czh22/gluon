#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8

import rospy
import torch
from std_msgs.msg import Float64
import math
import message_filters
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
import math
from scipy.spatial.transform import Rotation
import numpy as np
import torch.nn as nn
from geometry_msgs.msg import Vector3Stamped
import time

concat_num = 5
frame_len = 25


pub_r1 = 0
pub_r2 = 0
pub_r3 = 0
pub_r4 = 0
pub_r5 = 0
pub_r6 = 0
pub_l1 = 0
pub_l2 = 0
pub_l3 = 0
pub_l4 = 0
pub_l5 = 0
pub_l6 = 0
pub_cp = 0

# pub_r1 = rospy.Publisher('/gluon/right_joint1_position_controller/command',Float64,queue_size=10)
# pub_r2 = rospy.Publisher('/gluon/right_joint2_position_controller/command',Float64,queue_size=10)
# pub_r3 = rospy.Publisher('/gluon/right_joint3_position_controller/command',Float64,queue_size=10)
# pub_r4 = rospy.Publisher('/gluon/right_joint4_position_controller/command',Float64,queue_size=10)
# pub_r5 = rospy.Publisher('/gluon/right_joint5_position_controller/command',Float64,queue_size=10)
# pub_r6 = rospy.Publisher('/gluon/right_joint6_position_controller/command',Float64,queue_size=10)
# pub_l1 = rospy.Publisher('/gluon/left_joint1_position_controller/command',Float64,queue_size=10)
# pub_l2 = rospy.Publisher('/gluon/left_joint2_position_controller/command',Float64,queue_size=10)
# pub_l3 = rospy.Publisher('/gluon/left_joint3_position_controller/command',Float64,queue_size=10)
# pub_l4 = rospy.Publisher('/gluon/left_joint4_position_controller/command',Float64,queue_size=10)
# pub_l5 = rospy.Publisher('/gluon/left_joint5_position_controller/command',Float64,queue_size=10)
# pub_l6 = rospy.Publisher('/gluon/left_joint6_position_controller/command',Float64,queue_size=10)
# pub_cp = rospy.Publisher('/gluon/cloud_platform_joint_position_controller/command',Float64,queue_size=10)
msg_r1 = Float64()
msg_r2 = Float64()
msg_r3 = Float64()
msg_r4 = Float64()
msg_r5 = Float64()
msg_r6 = Float64()
msg_l1 = Float64()
msg_l2 = Float64()
msg_l3 = Float64()
msg_l4 = Float64()
msg_l5 = Float64()
msg_l6 = Float64()
msg_cp = Float64()

class my_model(nn.Module):
    def __init__(self, input_dim):
        super(my_model, self).__init__()
# TODO: modify model's structure, be aware of dimensions.
        self.layers = nn.Sequential(
        nn.Linear(input_dim, 128),

        nn.ReLU(),
        nn.Linear(128, 128),
        nn.ReLU(),
        nn.Linear(128, 128),
        nn.ReLU(),
        nn.Linear(128, 64),

        nn.ReLU(),
        nn.Linear(64, 32),

        nn.ReLU(),
        nn.Linear(32, 13)
        )

    def forward(self, x):
        x = self.layers(x)
        # x = x.squeeze(1) # (B, 1) -> (B)
        return x
    
model_path = '/home/s/Desktop/catkin_workspace/src/gluon/model/model_twistspeed5.ckpt'
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f'DEVICE: {device}')
model = my_model(input_dim=frame_len * concat_num - 13).to(device)
model.load_state_dict(torch.load(model_path))
print('model loaded')


def pub_all():
    pub_cp.publish(msg_cp)
    pub_l1.publish(msg_l1)
    pub_l2.publish(msg_l2)
    pub_l3.publish(msg_l3)
    pub_l4.publish(msg_l4)
    pub_l5.publish(msg_l5)
    pub_l6.publish(msg_l6)
    pub_r1.publish(msg_r1)
    pub_r2.publish(msg_r2)
    pub_r3.publish(msg_r3)
    pub_r4.publish(msg_r4)
    pub_r5.publish(msg_r5)
    pub_r6.publish(msg_r6)
    

def detect_bound():
    if msg_cp.data < 0:
        msg_cp.data = 0
    elif msg_cp.data > math.pi:
        msg_cp.data = math.pi 

    if msg_l1.data < -2.4:
        msg_l1.data = -2.4
    elif msg_l1.data > 2.4:
        msg_l1.data = 2.4 
    if msg_l2.data < -1.54:
        msg_l2.data = -1.54
    elif msg_l2.data > 0.1:
        msg_l2.data = 0.1 
    if msg_l3.data < -2.4:
        msg_l3.data = -2.4
    elif msg_l3.data > 2.4:
        msg_l3.data = 2.4 
    if msg_l4.data < -2.4:
        msg_l4.data = -2.4
    elif msg_l4.data > 2.4:
        msg_l4.data = 2.4 
    if msg_l5.data < -2.4:
        msg_l5.data = -2.4
    elif msg_l5.data > 2.4:
        msg_l5.data = 2.4 
    if msg_l6.data < -2.4:
        msg_l6.data = -2.4
    elif msg_l6.data > 2.4:
        msg_l6.data = 2.4 

    if msg_r1.data < -2.4:
        msg_r1.data = -2.4
    elif msg_r1.data > 2.4:
        msg_r1.data = 2.4 
    if msg_r2.data < -0.1:
        msg_r2.data = -0.1
    elif msg_r2.data > 1.54:
        msg_r2.data = 1.54 
    if msg_r3.data < -2.4:
        msg_r3.data = -2.4
    elif msg_r3.data > 2.4:
        msg_r3.data = 2.4 
    if msg_r4.data < -2.4:
        msg_r4.data = -2.4
    elif msg_r4.data > 2.4:
        msg_r4.data = 2.4 
    if msg_r5.data < -2.4:
        msg_r5.data = -2.4
    elif msg_r5.data > 2.4:
        msg_r5.data = 2.4 
    if msg_r6.data < -2.4:
        msg_r6.data = -2.4
    elif msg_r6.data > 2.4:
        msg_r6.data = 2.4 


def EulerAndQuaternionTransform( intput_data):
    """
        四元素与欧拉角互换
    """
    data_len = len(intput_data)
    angle_is_not_rad = False
 
    if data_len == 3:
        r = 0
        p = 0
        y = 0
        if angle_is_not_rad: # 180 ->pi
            r = math.radians(intput_data[0]) 
            p = math.radians(intput_data[1])
            y = math.radians(intput_data[2])
        else:
            r = intput_data[0] 
            p = intput_data[1]
            y = intput_data[2]
 
        sinp = math.sin(p/2)
        siny = math.sin(y/2)
        sinr = math.sin(r/2)
 
        cosp = math.cos(p/2)
        cosy = math.cos(y/2)
        cosr = math.cos(r/2)
 
        w = cosr*cosp*cosy + sinr*sinp*siny
        x = sinr*cosp*cosy - cosr*sinp*siny
        y = cosr*sinp*cosy + sinr*cosp*siny
        z = cosr*cosp*siny - sinr*sinp*cosy
        return [w,x,y,z]
 
    elif data_len == 4:
 
        w = intput_data[0] 
        x = intput_data[1]
        y = intput_data[2]
        z = intput_data[3]
 
        r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        p = math.asin(2 * (w * y - z * x))
        y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
 
        if angle_is_not_rad : # pi -> 180
            r = math.degrees(r)
            p = math.degrees(p)
            y = math.degrees(y)
        return [r,p,y]

def joint_init():
    
    msg_l1.data = 0
    msg_l2.data = 0
    msg_l3.data = 0
    msg_l4.data = 0
    msg_l5.data = 0
    msg_l6.data = 0
    msg_r1.data = 0
    msg_r2.data = 0
    msg_r3.data = 0
    msg_r4.data = 0
    msg_r5.data = 0
    msg_r6.data = 0
    msg_cp.data = math.pi/2

count = 0
x = torch.empty(frame_len * concat_num - 13).to(device)
frame = torch.empty(frame_len).to(device)

def callback(data1,data2,data3,data4,data5):
    print('callback')
    global count
    rolll, pitchl, yawl = EulerAndQuaternionTransform([data2.orientation.w, data2.orientation.x, data2.orientation.y, data2.orientation.z])
    rollr, pitchr, yawr = EulerAndQuaternionTransform([data4.orientation.w, data4.orientation.x, data4.orientation.y, data4.orientation.z])
    frame[0:3] = torch.tensor([rolll, pitchl, yawl])
    frame[3:6] = torch.tensor([data1.vector.x, data1.vector.y, data1.vector.z])
    frame[6:9] = torch.tensor([rollr, pitchr, yawr])
    frame[9:12] = torch.tensor([data3.vector.x, data3.vector.y, data3.vector.z])
    
    if count<concat_num-1:
        frame[12:] = torch.tensor([math.pi/2,0,0,0,0,0,0,0,0,0,0,0,0])

        x[count*frame_len:(count+1)*frame_len] = frame
        count+=1
    
    if count==concat_num-1:
        frame[12:] = torch.tensor([math.pi/2,0,0,0,0,0,0,0,0,0,0,0,0])

        x[count*frame_len:frame_len * concat_num - 13] = frame[:frame_len-13]
        count+=1

    if count>concat_num-1:
        frame[12:] = torch.tensor(data5.position)
        for i in range(concat_num-1):
            x[i*frame_len:(i+1)*frame_len-13] = x[(i+1)*frame_len:(i+2)*frame_len-13]

        for i in range(concat_num-2):
            x[(i+1)*frame_len-13:(i+1)*frame_len] = x[(i+2)*frame_len-13:(i+2)*frame_len]

        x[(concat_num-1)*frame_len:frame_len * concat_num - 13] = frame[:frame_len-13]
        x[(concat_num-1)*frame_len-13:(concat_num-1)*frame_len] = frame[frame_len-13:]

            
        y = model(x)
        # frame[12:] = y.to(device)
        
        
        

        is_left_speed0 = True
        for i in frame[3:6]:
            if i != 0:
                is_left_speed0 = False
        if is_left_speed0 == True:
            msg_cp.data = frame[12]
            msg_l1.data = frame[13]
            msg_l2.data = frame[14]
            msg_l3.data = frame[15]
            msg_l4.data = frame[16]
            msg_l5.data = frame[17]
            msg_l6.data = frame[18]
        else:
            msg_cp.data = y[0]
            msg_l1.data = y[1]
            msg_l2.data = y[2]
            msg_l3.data = y[3]
            msg_l4.data = y[4]
            msg_l5.data = y[5]
            msg_l6.data = y[6]


        is_right_speed0 = True
        for i in frame[9:12]:
            if i != 0:
                is_right_speed0 = False
        if is_right_speed0 == True:
            msg_r1.data = frame[19]
            msg_r2.data = frame[20]
            msg_r3.data = frame[21]
            msg_r4.data = frame[22]
            msg_r5.data = frame[23]
            msg_r6.data = frame[24]
        else:
            msg_r1.data = y[7]
            msg_r2.data = y[8]
            msg_r3.data = y[9]
            msg_r4.data = y[10]
            msg_r5.data = y[11]
            msg_r6.data = y[12]



        detect_bound()
        # frame[12:] = torch.tensor([msg_cp.data,msg_l1.data,msg_l2.data,msg_l3.data,msg_l4.data,msg_l5.data,msg_l6.data,msg_r1.data,msg_r2.data,msg_r3.data,msg_r4.data,msg_r5.data,msg_r6.data])
        pub_all()
    


def model_operation():
    rospy.init_node('model_operation',anonymous=True)
    global pub_cp
    global pub_l1
    global pub_l2
    global pub_l3
    global pub_l4
    global pub_l5
    global pub_l6
    global pub_r1
    global pub_r2
    global pub_r3
    global pub_r4
    global pub_r5
    global pub_r6

    pub_r1 = rospy.Publisher('/gluon/right_joint1_position_controller/command',Float64,queue_size=10)
    pub_r2 = rospy.Publisher('/gluon/right_joint2_position_controller/command',Float64,queue_size=10)
    pub_r3 = rospy.Publisher('/gluon/right_joint3_position_controller/command',Float64,queue_size=10)
    pub_r4 = rospy.Publisher('/gluon/right_joint4_position_controller/command',Float64,queue_size=10)
    pub_r5 = rospy.Publisher('/gluon/right_joint5_position_controller/command',Float64,queue_size=10)
    pub_r6 = rospy.Publisher('/gluon/right_joint6_position_controller/command',Float64,queue_size=10)
    pub_l1 = rospy.Publisher('/gluon/left_joint1_position_controller/command',Float64,queue_size=10)
    pub_l2 = rospy.Publisher('/gluon/left_joint2_position_controller/command',Float64,queue_size=10)
    pub_l3 = rospy.Publisher('/gluon/left_joint3_position_controller/command',Float64,queue_size=10)
    pub_l4 = rospy.Publisher('/gluon/left_joint4_position_controller/command',Float64,queue_size=10)
    pub_l5 = rospy.Publisher('/gluon/left_joint5_position_controller/command',Float64,queue_size=10)
    pub_l6 = rospy.Publisher('/gluon/left_joint6_position_controller/command',Float64,queue_size=10)
    pub_cp = rospy.Publisher('/gluon/cloud_platform_joint_position_controller/command',Float64,queue_size=10)

    
    t1 = message_filters.Subscriber("/left_speed", Vector3Stamped)
    t2 = message_filters.Subscriber("/left_no_g_imu", Imu)
    t3 = message_filters.Subscriber("/right_speed", Vector3Stamped)
    t4 = message_filters.Subscriber("/right_no_g_imu", Imu)
    t5 = message_filters.Subscriber("/realtime_joint", JointState)


    ts = message_filters.ApproximateTimeSynchronizer([t1,t2,t3,t4,t5], queue_size=10, slop=1, allow_headerless=True)
    
    # joint_init()
    # pub_all()
    # time.sleep(1)
    ts.registerCallback(callback)
    # r = rospy.Rate(50)
    print('ready')
    # while (1):
    #     pub_all()
    #     r.sleep()
    rospy.spin()



if __name__ == '__main__':
    model_operation()