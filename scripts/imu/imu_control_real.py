#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import rospy
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
import numpy as np
import message_filters
from sensor_msgs.msg import JointState
import math
from std_msgs.msg import Header


msgl = JointState()
msgr = JointState()


first_time = True
trans_matrixl1 = 0
trans_matrixl2 = 0
trans_matrixl3 = 0
trans_matrixr1 = 0
trans_matrixr2 = 0
trans_matrixr3 = 0

# to do: solve the risk which could be caused by boundary crossing
# def detect_cross_bound(data1,data2):




def solve_l1l2(l1_matrix):
    
    trans_l1_matrix = np.dot(trans_matrixl1, l1_matrix)
    trans_l1_rotation = Rotation.from_matrix(trans_l1_matrix)
    l1y, l1z, l1x = trans_l1_rotation.as_euler('YZX')
  

    return -l1z, l1x

def solve_r1r2(r1_matrix):
    
    trans_r1_matrix = np.dot(trans_matrixr1, r1_matrix)
    trans_r1_rotation = Rotation.from_matrix(trans_r1_matrix)
    r1y, r1z, r1x = trans_r1_rotation.as_euler('YZX')
  

    return -r1z, r1x




def solve_l3(l1_rotation, l2_matrix):
 
    trans_l2_matrix = np.dot(trans_matrixl2, l2_matrix)
    trans_l2_rotation = Rotation.from_matrix(trans_l2_matrix)
  
    trans_matrix = np.dot(np.linalg.inv(l1_rotation.as_matrix()),trans_l2_matrix)
    z,y,x = Rotation.from_matrix(trans_matrix).as_euler('ZYX')
   
    return -x

def solve_r3(r1_rotation, r2_matrix):
 
    trans_r2_matrix = np.dot(trans_matrixr2, r2_matrix)
    trans_r2_rotation = Rotation.from_matrix(trans_r2_matrix)
  
    trans_matrix = np.dot(np.linalg.inv(r1_rotation.as_matrix()),trans_r2_matrix)
    z,y,x = Rotation.from_matrix(trans_matrix).as_euler('ZYX')
   
    return -x


def solve_l4l5l6(l2_rotation, l3_matrix):
    
 
    trans_l3_matrix = np.dot(trans_matrixl3, l3_matrix)
    trans_l3_rotation = Rotation.from_matrix(trans_l3_matrix)
   
    trans_matrix = np.dot(np.linalg.inv(l2_rotation.as_matrix()),trans_l3_matrix)
    z,x,y = Rotation.from_matrix(trans_matrix).as_euler('ZXY')
    return x,y,0

def solve_r4r5r6(r2_rotation, r3_matrix):
    
 
    trans_r3_matrix = np.dot(trans_matrixr3, r3_matrix)
    trans_r3_rotation = Rotation.from_matrix(trans_r3_matrix)
   
    trans_matrix = np.dot(np.linalg.inv(r2_rotation.as_matrix()),trans_r3_matrix)
    z,x,y = Rotation.from_matrix(trans_matrix).as_euler('ZXY')
    return x,-y,0



def check_safety(pos):
    for i in range(6):
        if(abs(pos[i]-msgl.position[i])>0.01):
            if(pos[i]>msgl.position[i]):
                pos[i] = msgl.position[i]+0.01
            elif(pos[i]<msgl.position[i]):
                pos[i] = msgl.position[i]-0.01

    for i in range(6):
        if(abs(pos[i+6]-msgr.position[i])>0.01):
            if(pos[i+6]>msgr.position[i]):
                pos[i+6] = msgr.position[i]+0.01
            elif(pos[i+6]<msgr.position[i]):
                pos[i+6] = msgr.position[i]-0.01
    if pos[0] < -2.4:
        pos[0] = -2.4
    elif pos[0] > 2.4:
        pos[0] = 2.4 
    if pos[1] < -1.54:
        pos[1] = -1.54
    elif pos[1] > 0.1:
        pos[1] = 0.1 
    if pos[2] < -2.4:
        pos[2] = -2.4
    elif pos[2] > 2.4:
        pos[2] = 2.4 
    if pos[3] < -2.4:
        pos[3] = -2.4
    elif pos[3] > 2.4:
        pos[3] = 2.4 
    if pos[4] < -2.4:
        pos[4] = -2.4
    elif pos[4] > 2.4:
        pos[4] = 2.4 
    if pos[5] < -2.4:
        pos[5] = -2.4
    elif pos[5] > 2.4:
        pos[5] = 2.4 
    
    if pos[6] < -2.4:
        pos[6] = -2.4
    elif pos[6] > 2.4:
        pos[6] = 2.4 
    if pos[7] < -0.1:
        pos[7] = -0.1
    elif pos[7] > 1.54:
        pos[7] = 1.54 
    if pos[8] < -2.4:
        pos[8] = -2.4
    elif pos[8] > 2.4:
        pos[8] = 2.4 
    if pos[9] < -2.4:
        pos[9] = -2.4
    elif pos[9] > 2.4:
        pos[9] = 2.4 
    if pos[10] < -2.4:
        pos[10] = -2.4
    elif pos[10] > 2.4:
        pos[10] = 2.4 
    if pos[11] < -2.4:
        pos[11] = -2.4
    elif pos[11] > 2.4:
        pos[11] = 2.4 
    return pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6],pos[7],pos[8],pos[9],pos[10],pos[11]
            




def callback(data1,data2,data3,data4,data5,data6):
    global first_time
    global trans_matrixl1
    global trans_matrixl2
    global trans_matrixl3
    global trans_matrixr1
    global trans_matrixr2
    global trans_matrixr3
    print("hello")
    
    l1_quaternion = [data1.orientation.x, data1.orientation.y, data1.orientation.z, data1.orientation.w]
    l2_quaternion = [data2.orientation.x, data2.orientation.y, data2.orientation.z, data2.orientation.w]
    l3_quaternion = [data3.orientation.x, data3.orientation.y, data3.orientation.z, data3.orientation.w]
    r1_quaternion = [data4.orientation.x, data4.orientation.y, data4.orientation.z, data4.orientation.w]
    r2_quaternion = [data5.orientation.x, data5.orientation.y, data5.orientation.z, data5.orientation.w]
    r3_quaternion = [data6.orientation.x, data6.orientation.y, data6.orientation.z, data6.orientation.w]
    l1_rotation = Rotation.from_quat(l1_quaternion)
    l2_rotation = Rotation.from_quat(l2_quaternion)
    l3_rotation = Rotation.from_quat(l3_quaternion)
    r1_rotation = Rotation.from_quat(r1_quaternion)
    r2_rotation = Rotation.from_quat(r2_quaternion)
    r3_rotation = Rotation.from_quat(r3_quaternion)
    l1_matrix = l1_rotation.as_matrix()
    l2_matrix = l2_rotation.as_matrix()
    l3_matrix = l3_rotation.as_matrix()
    r1_matrix = r1_rotation.as_matrix()
    r2_matrix = r2_rotation.as_matrix()
    r3_matrix = r3_rotation.as_matrix()
    if first_time == True:
        trans_matrixl1 = np.linalg.inv(l1_matrix)
        trans_matrixl2 = np.dot(l1_matrix, np.linalg.inv(l2_matrix))
        trans_matrixl3 = np.dot(l2_matrix, np.linalg.inv(l3_matrix))
        trans_matrixr1 = np.linalg.inv(r1_matrix)
        trans_matrixr2 = np.dot(r1_matrix, np.linalg.inv(r2_matrix))
        trans_matrixr3 = np.dot(r2_matrix, np.linalg.inv(r3_matrix))
        first_time = False


    posl1, posl2 = solve_l1l2(l1_matrix)
    posl3 = solve_l3(l1_rotation, l2_matrix)
    posl4, posl5, posl6 = solve_l4l5l6(l2_rotation, l3_matrix)

    posr1, posr2 = solve_r1r2(r1_matrix)
    posr3 = solve_r3(r1_rotation, r2_matrix)
    posr4, posr5, posr6 = solve_r4r5r6(r2_rotation, r3_matrix)

    posl5-=math.pi/2
    posr5+=math.pi/2

    posl1,posl2,posl3,posl4,posl5,posl6,posr1,posr2,posr3,posr4,posr5,posr6 = check_safety([posl1,posl2,posl3,posl4,posl5,posl6,posr1,posr2,posr3,posr4,posr5,posr6])
    msgl.position = [posl1,posl2,posl3,posl4,posl5,posl6]
    msgr.position = [posr1,posr2,posr3,posr4,posr5,posr6]
    msgl.header.stamp = rospy.Time.now()
    msgr.header.stamp = rospy.Time.now()
    publ.publish(msgl)
    pubr.publish(msgr)


def imu_control_real():
    rospy.init_node('imu_control_real')
    msgl.header = Header()
    global publ
    global pubr
    publ = rospy.Publisher("/left_joint_states", JointState, queue_size=10)
    pubr = rospy.Publisher("/right_joint_states", JointState, queue_size=10)
    msgl.name = ["left_axis_joint_1","left_axis_joint_2","left_axis_joint_3","left_axis_joint_4","left_axis_joint_5","left_axis_joint_6"]
    msgl.position = [0,0,0,0,0,0]
    msgr.name = ["right_axis_joint_1","right_axis_joint_2","right_axis_joint_3","right_axis_joint_4","right_axis_joint_5","right_axis_joint_6"]
    msgr.position = [0,0,0,0,0,0]
    t1 = message_filters.Subscriber("/l1_imu", Imu)
    t2 = message_filters.Subscriber("/l2_imu", Imu)
    t3 = message_filters.Subscriber("/l3_imu", Imu)
    t4 = message_filters.Subscriber("/r1_imu", Imu)
    t5 = message_filters.Subscriber("/r2_imu", Imu)
    t6 = message_filters.Subscriber("/r3_imu", Imu)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2, t3, t4, t5, t6], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    imu_control_real()