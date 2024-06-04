#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped


msgl = Vector3Stamped()
msgr = Vector3Stamped()
frequency = 100.0
static_countl = 0
if_staticl = True
static_countr = 0
if_staticr = True


def detect_staticl():
    global if_staticl


    if static_countl > 10:
        if_staticl = True

    if if_staticl == True:
        msgl.vector.x = 0
        msgl.vector.y = 0
        msgl.vector.z = 0
    





def left_callback(data):
    global if_staticl
    global static_countl
    msgl.vector.x += data.linear_acceleration.x/frequency
    msgl.vector.y += data.linear_acceleration.y/frequency
    msgl.vector.z += data.linear_acceleration.z/frequency
    if ((abs(data.linear_acceleration.x) > 0.5) or (abs(data.linear_acceleration.y) > 0.5)) or (abs(data.linear_acceleration.z) > 0.5):
        if_staticl = False
        static_countl = 0
    else:
        static_countl+=1
    print()
    print('left '+str(if_staticl))
    detect_staticl()


def detect_staticr():
    global if_staticr


    if static_countr > 10:
        if_staticr = True

    if if_staticr == True:
        msgr.vector.x = 0
        msgr.vector.y = 0
        msgr.vector.z = 0
    





def right_callback(data):
    global if_staticr
    global static_countr
    msgr.vector.x += data.linear_acceleration.x/frequency
    msgr.vector.y += data.linear_acceleration.y/frequency
    msgr.vector.z += data.linear_acceleration.z/frequency
    if ((abs(data.linear_acceleration.x) > 0.5) or (abs(data.linear_acceleration.y) > 0.5)) or (abs(data.linear_acceleration.z) > 0.5):
        if_staticr = False
        static_countr = 0
    else:
        static_countr+=1
    print('right'+str(if_staticr))
    detect_staticr()
    



def speed_integration():
    rospy.init_node('speed_integration', anonymous=True)
    msgl.vector.x = 0
    msgl.vector.y = 0
    msgl.vector.z = 0
    msgr.vector.x = 0
    msgr.vector.y = 0
    msgr.vector.z = 0
    msgl.header.frame_id = 'left'
    msgr.header.frame_id = 'right'
    rospy.Subscriber("/left_no_g_imu", Imu, left_callback)
    rospy.Subscriber("/right_no_g_imu", Imu, right_callback)
    global publ
    global pubr
    publ = rospy.Publisher('/left_speed', Vector3Stamped, queue_size=10)
    pubr = rospy.Publisher('/right_speed', Vector3Stamped, queue_size=10)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        msgl.header.stamp = rospy.Time.now()
        msgr.header.stamp = rospy.Time.now()
        publ.publish(msgl)
        pubr.publish(msgr)
        rate.sleep()
    



if __name__ == '__main__':
    speed_integration()