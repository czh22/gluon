#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import rospy
from sensor_msgs.msg import JointState
import time
import rosparam

rosparam.set_param('use_sim_time', 'False')


def callback(data):
    
    data.header.stamp = rospy.get_rostime()
    # print(data.header.stamp)
    pub.publish(data)

def realtime_joint():
    rospy.init_node('realtime_joint')
    print(rospy.Time.now())
    
    rospy.Subscriber('/gluon/joint_states', JointState, callback)
    global pub
    pub = rospy.Publisher('/realtime_joint', JointState, queue_size=10)
    rospy.spin()

if __name__=='__main__':
    realtime_joint()