#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import rospy
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
import numpy as np
import message_filters
from std_msgs.msg import Float64
import math
import tf

msg_l1 = Float64()
msg_l2 = Float64()
msg_l3 = Float64()
msg_l4 = Float64()
msg_l5 = Float64()
msg_l6 = Float64()
msg_r1 = Float64()
msg_r2 = Float64()
msg_r3 = Float64()
msg_r4 = Float64()
msg_r5 = Float64()
msg_r6 = Float64()


    
first_time = True
trans_matrixl1 = 0
trans_matrixl2 = 0
trans_matrixl3 = 0
trans_matrixr1 = 0
trans_matrixr2 = 0
trans_matrixr3 = 0



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
    # br = tf.TransformBroadcaster()
    # br.sendTransform((0,-1,0),(trans_l2_rotation.as_quat()),rospy.Time.now(),"l2","map")
    # br.sendTransform((0,-1,0),(Rotation.from_matrix(trans_matrix).as_quat()),rospy.Time.now(),"l2re","l1")
    # br = tf.TransformBroadcaster()
    # br.sendTransform((0,0,0),(l1_rotation.as_quat),rospy.Time.now(),"l2","world")
    # br.sendTransform((0,-1,0),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"l2","world")
    return -x

def solve_r3(r1_rotation, r2_matrix):
 
    trans_r2_matrix = np.dot(trans_matrixr2, r2_matrix)
    trans_r2_rotation = Rotation.from_matrix(trans_r2_matrix)
  
    trans_matrix = np.dot(np.linalg.inv(r1_rotation.as_matrix()),trans_r2_matrix)
    z,y,x = Rotation.from_matrix(trans_matrix).as_euler('ZYX')
    # br = tf.TransformBroadcaster()
    # br.sendTransform((0,-1,0),(trans_l2_rotation.as_quat()),rospy.Time.now(),"l2","map")
    # br.sendTransform((0,-1,0),(Rotation.from_matrix(trans_matrix).as_quat()),rospy.Time.now(),"l2re","l1")
    # br = tf.TransformBroadcaster()
    # br.sendTransform((0,0,0),(l1_rotation.as_quat),rospy.Time.now(),"l2","world")
    # br.sendTransform((0,-1,0),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"l2","world")
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

   






def callback(data1,data2,data3,data4,data5,data6):
    global first_time
    global trans_matrixl1
    global trans_matrixl2
    global trans_matrixl3
    global trans_matrixr1
    global trans_matrixr2
    global trans_matrixr3
    # print('callback')
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
    # br = tf.TransformBroadcaster()
    # br.sendTransform((0,0,0),(l1_quaternion),rospy.Time.now(),"l1","map")
    
    # br.sendTransform((0,-2,0),(l3_quaternion),rospy.Time.now(),"l3","map")
    if first_time == True:
        trans_matrixl1 = np.linalg.inv(l1_matrix)
        trans_matrixl2 = np.dot(l1_matrix, np.linalg.inv(l2_matrix))
        trans_matrixl3 = np.dot(l2_matrix, np.linalg.inv(l3_matrix))
        trans_matrixr1 = np.linalg.inv(r1_matrix)
        trans_matrixr2 = np.dot(r1_matrix, np.linalg.inv(r2_matrix))
        trans_matrixr3 = np.dot(r2_matrix, np.linalg.inv(r3_matrix))
        first_time = False


    msg_l1.data, msg_l2.data = solve_l1l2(l1_matrix)
    msg_l3.data = solve_l3(l1_rotation, l2_matrix)
    msg_l4, msg_l5, msg_l6 = solve_l4l5l6(l2_rotation, l3_matrix)

    msg_r1.data, msg_r2.data = solve_r1r2(r1_matrix)
    msg_r3.data = solve_r3(r1_rotation, r2_matrix)
    msg_r4, msg_r5, msg_r6 = solve_r4r5r6(r2_rotation, r3_matrix)
    
    # msg_l5.data -= math.pi/2
    # msg_r5.data += math.pi/2

    publ1.publish(msg_l1)
    publ2.publish(msg_l2)
    publ3.publish(msg_l3)
    publ4.publish(msg_l4)
    publ5.publish(msg_l5)
    publ6.publish(msg_l6)

    pubr1.publish(msg_r1)
    pubr2.publish(msg_r2)
    pubr3.publish(msg_r3)
    pubr4.publish(msg_r4)
    pubr5.publish(msg_r5)
    pubr6.publish(msg_r6)





def gazebo_test2():
    rospy.init_node('gazebo_test2')
    global publ1
    global publ2
    global publ3
    global publ4
    global publ5
    global publ6
    global pubr1
    global pubr2
    global pubr3
    global pubr4
    global pubr5
    global pubr6
    publ1 = rospy.Publisher("/gluon/left_joint1_position_controller/command", Float64, queue_size=10)
    publ2 = rospy.Publisher("/gluon/left_joint2_position_controller/command", Float64, queue_size=10)
    publ3 = rospy.Publisher("/gluon/left_joint3_position_controller/command", Float64, queue_size=10)
    publ4 = rospy.Publisher("/gluon/left_joint4_position_controller/command", Float64, queue_size=10)
    publ5 = rospy.Publisher("/gluon/left_joint5_position_controller/command", Float64, queue_size=10)
    publ6 = rospy.Publisher("/gluon/left_joint6_position_controller/command", Float64, queue_size=10)
    pubr1 = rospy.Publisher("/gluon/right_joint1_position_controller/command", Float64, queue_size=10)
    pubr2 = rospy.Publisher("/gluon/right_joint2_position_controller/command", Float64, queue_size=10)
    pubr3 = rospy.Publisher("/gluon/right_joint3_position_controller/command", Float64, queue_size=10)
    pubr4 = rospy.Publisher("/gluon/right_joint4_position_controller/command", Float64, queue_size=10)
    pubr5 = rospy.Publisher("/gluon/right_joint5_position_controller/command", Float64, queue_size=10)
    pubr6 = rospy.Publisher("/gluon/right_joint6_position_controller/command", Float64, queue_size=10)
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
    gazebo_test2()