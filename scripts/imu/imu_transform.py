#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import rospy
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
import numpy as np
import math
from geometry_msgs.msg import Vector3
import message_filters


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

def quaternion2rot(quaternion):
    r = Rotation.from_quat(quaternion)
    rot = r.as_matrix()
    return rot


def euler2rot(euler):
    r = Rotation.from_euler('xyz', euler, degrees=False)
    rotation_matrix = r.as_matrix()
    return rotation_matrix


def left_callback(data1):
    temp = data1.orientation.x
    data1.orientation.x = data1.orientation.y
    data1.orientation.y = temp
    temp = data1.orientation.z
    data1.orientation.z = data1.orientation.w
    data1.orientation.w = temp
    data1.linear_acceleration.y = -data1.linear_acceleration.y
    data1.linear_acceleration.z = -data1.linear_acceleration.z
    quaternion = [data1.orientation.x, data1.orientation.y, data1.orientation.z, data1.orientation.w]
    r, p, y = EulerAndQuaternionTransform(quaternion)
    data1.angular_velocity.x = r
    data1.angular_velocity.y = p
    data1.angular_velocity.z = y
    
    # euler = [data2.x, data2.y, data2.z]
    # rot = euler2rot(euler)
    rot = quaternion2rot(quaternion)
    g = np.array([[data1.linear_acceleration.x],[data1.linear_acceleration.y],[data1.linear_acceleration.z]])
    # trans_g = np.dot(np.linalg.inv(rot), g)
    trans_g = np.dot(rot, g)
    # data1.linear_acceleration.x = -data1.linear_acceleration.x
    # data1.linear_acceleration.y = -data1.linear_acceleration.y
    # data1.linear_acceleration.z = -data1.linear_acceleration.z
    data1.linear_acceleration.x = trans_g[0,0]
    data1.linear_acceleration.y = trans_g[1,0]
    data1.linear_acceleration.z = trans_g[2,0]-9.8
    # data1.angular_velocity.x = trans_g[0,0]
    # data1.angular_velocity.y = trans_g[1,0]
    # data1.angular_velocity.z = trans_g[2,0]
    # print(data1.linear_acceleration.x*data1.linear_acceleration.x+data1.linear_acceleration.y*data1.linear_acceleration.y+data1.linear_acceleration.z*data1.linear_acceleration.z)
    
    publ.publish(data1)


def right_callback(data1):
    temp = data1.orientation.x
    data1.orientation.x = data1.orientation.y
    data1.orientation.y = temp
    temp = data1.orientation.z
    data1.orientation.z = data1.orientation.w
    data1.orientation.w = temp
    data1.linear_acceleration.y = -data1.linear_acceleration.y
    data1.linear_acceleration.z = -data1.linear_acceleration.z
    quaternion = [data1.orientation.x, data1.orientation.y, data1.orientation.z, data1.orientation.w]
    r, p, y = EulerAndQuaternionTransform(quaternion)
    data1.angular_velocity.x = r
    data1.angular_velocity.y = p
    data1.angular_velocity.z = y
    
    # euler = [data2.x, data2.y, data2.z]
    # rot = euler2rot(euler)
    rot = quaternion2rot(quaternion)
    g = np.array([[data1.linear_acceleration.x],[data1.linear_acceleration.y],[data1.linear_acceleration.z]])
    # trans_g = np.dot(np.linalg.inv(rot), g)
    trans_g = np.dot(rot, g)
    # data1.linear_acceleration.x = -data1.linear_acceleration.x
    # data1.linear_acceleration.y = -data1.linear_acceleration.y
    # data1.linear_acceleration.z = -data1.linear_acceleration.z
    data1.linear_acceleration.x = trans_g[0,0]
    data1.linear_acceleration.y = trans_g[1,0]
    data1.linear_acceleration.z = trans_g[2,0]-9.8
    # data1.angular_velocity.x = trans_g[0,0]
    # data1.angular_velocity.y = trans_g[1,0]
    # data1.angular_velocity.z = trans_g[2,0]
    # print(data1.linear_acceleration.x*data1.linear_acceleration.x+data1.linear_acceleration.y*data1.linear_acceleration.y+data1.linear_acceleration.z*data1.linear_acceleration.z)
    
    pubr.publish(data1)
    



def imu_transform():
    rospy.init_node('imu_transform')

    # t1= message_filters.Subscriber("/imu", Imu)
    # t2 =message_filters.Subscriber("/euler_angles", Vector3)
    # ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1, allow_headerless=True)
    # ts.registerCallback(callback)
    global publ
    global pubr
    publ = rospy.Publisher("/left_no_g_imu", Imu, queue_size=10)
    pubr = rospy.Publisher("/right_no_g_imu", Imu, queue_size=10)
    
    rospy.Subscriber('/left_imu',Imu,left_callback)
    rospy.Subscriber('/right_imu',Imu,right_callback)
    
    rospy.spin()

if __name__ == '__main__':
    imu_transform()