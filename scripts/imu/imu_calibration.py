#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import rospy
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
import numpy as np
import math
from geometry_msgs.msg import Vector3
import message_filters


def EulerAndQuaternionTransform(intput_data):
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
    
first_time = True
trans_matrix = 0

def callback(data1,data2):
    global first_time
    global trans_matrix
    
    left_quaternion = [data1.orientation.x, data1.orientation.y, data1.orientation.z, data1.orientation.w]
    right_quaternion = [data2.orientation.x, data2.orientation.y, data2.orientation.z, data2.orientation.w]
    # lr, lp, ly = EulerAndQuaternionTransform(left_quaternion)
    left_rotation = Rotation.from_quat(left_quaternion)
    right_rotation = Rotation.from_quat(right_quaternion)
    left_matrix = left_rotation.as_matrix()
    right_matrix = right_rotation.as_matrix()
    if first_time == True:
        
        trans_matrix = np.dot(left_matrix, np.linalg.inv(right_matrix))
        first_time = False
    
    trans_right_matrix = np.dot(trans_matrix, right_matrix)
    trans_right_rotation = Rotation.from_matrix(trans_right_matrix)
    lx, lz, ly = left_rotation.as_euler('XZY')
    data1.angular_velocity.x = lx
    data1.angular_velocity.y = ly
    data1.angular_velocity.z = lz
    # rr, rp, ry = EulerAndQuaternionTransform(right_quaternion)
    
    rx, rz, ry = trans_right_rotation.as_euler('XZY')
    data2.angular_velocity.x = rx
    data2.angular_velocity.y = ry
    data2.angular_velocity.z = rz
    publ.publish(data1)
    pubr.publish(data2)


def imu_calibration():
    rospy.init_node('imu_calibration')
    global publ
    global pubr
    publ = rospy.Publisher("/left_calibrated_imu", Imu, queue_size=10)
    pubr = rospy.Publisher("/right_calibrated_imu", Imu, queue_size=10)
    t1= message_filters.Subscriber("/l1_imu", Imu)
    t2 =message_filters.Subscriber("/l2_imu", Imu)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()
    


if __name__ == '__main__':
    imu_calibration()