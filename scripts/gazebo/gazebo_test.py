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



speed_x1 = 0
speed_y1 = 0
speed_z1 = 0
speed_x2 = 0
speed_y2 = 0
speed_z2 = 0
frequency = 50.0


static_count1 = 0
if_static1 = True
static_count2 = 0
if_static2 = True

def detect_static1():
    global if_static1
    global speed_x1
    global speed_y1
    global speed_z1
    


    if static_count1 > 5:
        if_static1 = True

    if if_static1 == True:
        speed_x1 = 0
        speed_y1 = 0
        speed_z1 = 0

def detect_static2():
    global if_static2
    global speed_x2
    global speed_y2
    global speed_z2
    


    if static_count2 > 5:
        if_static2 = True

    if if_static2 == True:
        speed_x2 = 0
        speed_y2 = 0
        speed_z2 = 0
     

def quaternion2rot(quaternion):
    r = Rotation.from_quat(quaternion)
    rot = r.as_matrix()
    return rot



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

def callback(data1,data2,data3):
    print('callback')
    global speed_x1
    global speed_y1
    global speed_z1
    global speed_x2
    global speed_y2
    global speed_z2
    global if_static1
    global static_count1
    global if_static2
    global static_count2
    global count
    roll1, pitch1, yaw1 = EulerAndQuaternionTransform([data1.orientation.w, data1.orientation.x, data1.orientation.y, data1.orientation.z])
    roll2, pitch2, yaw2 = EulerAndQuaternionTransform([data2.orientation.w, data2.orientation.x, data2.orientation.y, data2.orientation.z])
    quaternion1 = [data1.orientation.x, data1.orientation.y, data1.orientation.z, data1.orientation.w]
    quaternion2 = [data2.orientation.x, data2.orientation.y, data2.orientation.z, data2.orientation.w]
    rot1 = quaternion2rot(quaternion1)
    rot2 = quaternion2rot(quaternion2)
    g1 = np.array([[data1.linear_acceleration.x],[data1.linear_acceleration.y],[data1.linear_acceleration.z]])
    g2 = np.array([[data2.linear_acceleration.x],[data2.linear_acceleration.y],[data2.linear_acceleration.z]])

    trans_g1 = np.dot(rot1, g1)
    trans_g2 = np.dot(rot2, g2)
    data1.linear_acceleration.x = trans_g1[0,0]
    data1.linear_acceleration.y = trans_g1[1,0]
    data1.linear_acceleration.z = trans_g1[2,0]-9.8
    print('imu1 '+str(data1.linear_acceleration.x)+' '+str(data1.linear_acceleration.y)+' '+str(data1.linear_acceleration.z)+'\n')
    data2.linear_acceleration.x = trans_g2[0,0]
    data2.linear_acceleration.y = trans_g2[1,0]
    data2.linear_acceleration.z = trans_g2[2,0]-9.8
    print('imu2 '+str(data2.linear_acceleration.x)+' '+str(data2.linear_acceleration.y)+' '+str(data2.linear_acceleration.z)+'\n')
    speed_x1 += data1.linear_acceleration.x/frequency
    speed_y1 += data1.linear_acceleration.y/frequency
    speed_z1 += data1.linear_acceleration.z/frequency
    speed_x2 += data2.linear_acceleration.x/frequency
    speed_y2 += data2.linear_acceleration.y/frequency
    speed_z2 += data2.linear_acceleration.z/frequency
    # print('imu1 '+str(speed_x1)+' '+str(speed_y1)+' '+str(speed_z1)+'\n')
    if ((abs(data1.linear_acceleration.x) > 1) or (abs(data1.linear_acceleration.y) > 1)) or (abs(data1.linear_acceleration.z) > 1):
        if_static1 = False
        static_count1 = 0
    else:
        static_count1+=1
    print(if_static1)
    detect_static1()
    if ((abs(data2.linear_acceleration.x) > 1) or (abs(data2.linear_acceleration.y) > 1)) or (abs(data2.linear_acceleration.z) > 1):
        if_static2 = False
        static_count2 = 0
    else:
        static_count2+=1
    print(if_static2)
    detect_static2()
    # rolll, pitchl, yawl = EulerAndQuaternionTransform([data2.orientation.w, data2.orientation.x, data2.orientation.y, data2.orientation.z])
    # rollr, pitchr, yawr = EulerAndQuaternionTransform([data4.orientation.w, data4.orientation.x, data4.orientation.y, data4.orientation.z])
    frame[0:3] = torch.tensor([roll1, pitch1, yaw1])
    frame[3:6] = torch.tensor([speed_x1, speed_y1, speed_z1])
    frame[6:9] = torch.tensor([roll2, pitch2, yaw2])
    frame[9:12] = torch.tensor([speed_x2, speed_y2, speed_z2])
    
    if count<concat_num-1:
        frame[12:] = torch.tensor([math.pi/2,0,0,0,0,0,0,0,0,0,0,0,0])

        x[count*frame_len:(count+1)*frame_len] = frame
        count+=1
    
    if count==concat_num-1:
        frame[12:] = torch.tensor([math.pi/2,0,0,0,0,0,0,0,0,0,0,0,0])

        x[count*frame_len:frame_len * concat_num - 13] = frame[:frame_len-13]
        count+=1

    if count>concat_num-1:
        # frame[12:] = torch.tensor(data3.position)
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
        frame[12:] = torch.tensor([msg_cp.data,msg_l1.data,msg_l2.data,msg_l3.data,msg_l4.data,msg_l5.data,msg_l6.data,msg_r1.data,msg_r2.data,msg_r3.data,msg_r4.data,msg_r5.data,msg_r6.data])
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

    pub_r1 = rospy.Publisher('/gluon2/right_joint1_position_controller/command',Float64,queue_size=10)
    pub_r2 = rospy.Publisher('/gluon2/right_joint2_position_controller/command',Float64,queue_size=10)
    pub_r3 = rospy.Publisher('/gluon2/right_joint3_position_controller/command',Float64,queue_size=10)
    pub_r4 = rospy.Publisher('/gluon2/right_joint4_position_controller/command',Float64,queue_size=10)
    pub_r5 = rospy.Publisher('/gluon2/right_joint5_position_controller/command',Float64,queue_size=10)
    pub_r6 = rospy.Publisher('/gluon2/right_joint6_position_controller/command',Float64,queue_size=10)
    pub_l1 = rospy.Publisher('/gluon2/left_joint1_position_controller/command',Float64,queue_size=10)
    pub_l2 = rospy.Publisher('/gluon2/left_joint2_position_controller/command',Float64,queue_size=10)
    pub_l3 = rospy.Publisher('/gluon2/left_joint3_position_controller/command',Float64,queue_size=10)
    pub_l4 = rospy.Publisher('/gluon2/left_joint4_position_controller/command',Float64,queue_size=10)
    pub_l5 = rospy.Publisher('/gluon2/left_joint5_position_controller/command',Float64,queue_size=10)
    pub_l6 = rospy.Publisher('/gluon2/left_joint6_position_controller/command',Float64,queue_size=10)
    pub_cp = rospy.Publisher('/gluon2/cloud_platform_joint_position_controller/command',Float64,queue_size=10)

    
    t1= message_filters.Subscriber("/gluon1/gluon/left_imu", Imu)
    t2= message_filters.Subscriber("/gluon1/gluon/right_imu", Imu)
    t3 = message_filters.Subscriber("/gluon2/joint_states", JointState)


    ts = message_filters.ApproximateTimeSynchronizer([t1,t2,t3], queue_size=10, slop=1, allow_headerless=True)
    
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