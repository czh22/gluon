#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import rospy
import message_filters
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
import math
from scipy.spatial.transform import Rotation
import numpy as np


filename = '/home/s/Desktop/catkin_workspace/src/gluon/data/data.txt'
f = open(filename,'w') # 如果filename不存在会自动创建， 'w'表示写数据，写之前会清空文件中的原有数据！


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
    


    if static_count1 > 10:
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
    


    if static_count2 > 10:
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

def callback(data1, data2, data3):
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
    # print('imu1 '+str(data1.linear_acceleration.x)+' '+str(data1.linear_acceleration.y)+' '+str(data1.linear_acceleration.z)+'\n')
    data2.linear_acceleration.x = trans_g2[0,0]
    data2.linear_acceleration.y = trans_g2[1,0]
    data2.linear_acceleration.z = trans_g2[2,0]-9.8
    speed_x1 += data1.linear_acceleration.x/frequency
    speed_y1 += data1.linear_acceleration.y/frequency
    speed_z1 += data1.linear_acceleration.z/frequency
    speed_x2 += data2.linear_acceleration.x/frequency
    speed_y2 += data2.linear_acceleration.y/frequency
    speed_z2 += data2.linear_acceleration.z/frequency
    print('imu1 '+str(speed_x1)+' '+str(speed_y1)+' '+str(speed_z1)+'\n')
    if ((abs(data1.linear_acceleration.x) > 0.2) or (abs(data1.linear_acceleration.y) > 0.2)) or (abs(data1.linear_acceleration.z) > 0.2):
        if_static1 = False
        static_count1 = 0
    else:
        static_count1+=1
    print(if_static1)
    detect_static1()
    if ((abs(data2.linear_acceleration.x) > 0.2) or (abs(data2.linear_acceleration.y) > 0.2)) or (abs(data2.linear_acceleration.z) > 0.2):
        if_static2 = False
        static_count2 = 0
    else:
        static_count2+=1
    print(if_static2)
    detect_static2()
    f.write(str(round(roll1,4)))
    f.write(" ")
    f.write(str(round(pitch1,4)))
    f.write(" ")
    f.write(str(round(yaw1,4)))
    f.write(" ")
    # f.write(str(round(data1.angular_velocity.x,4)))
    # f.write(" ")
    # f.write(str(round(data1.angular_velocity.y,4)))
    # f.write(" ")
    # f.write(str(round(data1.angular_velocity.z,4)))
    # f.write(" ")
    f.write(str(round(speed_x1,4)))
    f.write(" ")
    f.write(str(round(speed_y1,4)))
    f.write(" ")
    f.write(str(round(speed_z1,4)))
    f.write(" ")
    f.write(str(round(roll2,4)))
    f.write(" ")
    f.write(str(round(pitch2,4)))
    f.write(" ")
    f.write(str(round(yaw2,4)))
    f.write(" ")
    # f.write(str(round(data2.angular_velocity.x,4)))
    # f.write(" ")
    # f.write(str(round(data2.angular_velocity.y,4)))
    # f.write(" ")
    # f.write(str(round(data2.angular_velocity.z,4)))
    # f.write(" ")
    f.write(str(round(speed_x2,4)))
    f.write(" ")
    f.write(str(round(speed_y2,4)))
    f.write(" ")
    f.write(str(round(speed_z2,4)))
    f.write(" ")
    f.write(str(round(data3.position[0],4)))
    f.write(" ")
    f.write(str(round(data3.position[1],4)))
    f.write(" ")
    f.write(str(round(data3.position[2],4)))
    f.write(" ")
    f.write(str(round(data3.position[3],4)))
    f.write(" ")
    f.write(str(round(data3.position[4],4)))
    f.write(" ")
    f.write(str(round(data3.position[5],4)))
    f.write(" ")
    f.write(str(round(data3.position[6],4)))
    f.write(" ")
    f.write(str(round(data3.position[7],4)))
    f.write(" ")
    f.write(str(round(data3.position[8],4)))
    f.write(" ")
    f.write(str(round(data3.position[9],4)))
    f.write(" ")
    f.write(str(round(data3.position[10],4)))
    f.write(" ")
    f.write(str(round(data3.position[11],4)))
    f.write(" ")
    f.write(str(round(data3.position[12],4)))
    f.write("\n")


def record_gazebo_data():
    rospy.init_node('record_gazebo_data', anonymous=True)
    t1= message_filters.Subscriber("/gluon/left_imu", Imu)
    t2= message_filters.Subscriber("/gluon/right_imu", Imu)
    t3 =message_filters.Subscriber("/gluon/joint_states", JointState)


    ts = message_filters.ApproximateTimeSynchronizer([t1, t2, t3], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()



if __name__ == '__main__':
    record_gazebo_data()