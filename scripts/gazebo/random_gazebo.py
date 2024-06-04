#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import rospy
from std_msgs.msg import Float64
import math
import random


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
    
def random_add():
    msg_l1.data += (random.random()-0.5)/3.0
    msg_l2.data += (random.random()-0.5)/3.0
    msg_l3.data += (random.random()-0.5)/3.0
    msg_l4.data += (random.random()-0.5)/3.0
    msg_l5.data += (random.random()-0.5)/3.0
    msg_l6.data += (random.random()-0.5)/3.0
    msg_r1.data += (random.random()-0.5)/3.0
    msg_r2.data += (random.random()-0.5)/3.0
    msg_r3.data += (random.random()-0.5)/3.0
    msg_r4.data += (random.random()-0.5)/3.0
    msg_r5.data += (random.random()-0.5)/3.0
    msg_l6.data += (random.random()-0.5)/3.0
    msg_cp.data += (random.random()-0.5)/3.0
    detect_bound()


def random_gazebo():

    rospy.init_node('random_gazebo', anonymous=True)
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
    rate = rospy.Rate(5)
    joint_init()
    count = 0
    while(1):
        random_add()
        pub_all()
        count += 1
        if count > 100:
            joint_init()
            count = 0
        rate.sleep()
        pub_all()
        if count > 100:
            joint_init()
            count = 0
        count += 1
    
    


if __name__ == '__main__':
    random_gazebo()