#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


from geometry_msgs.msg import Vector3
import rospy
import tf


msg = Vector3()
frequency = 50.0
br = tf.TransformBroadcaster()
msg.x = 0
msg.y = 0
msg.z = 0

def callback(data):

    msg.x -= data.x/frequency
    msg.y -= data.y/frequency
    msg.z -= data.z/frequency
    pub.publish(msg)
    br.sendTransform((msg.x,msg.y,msg.z),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"imu_position","world")



def position_integration():
    rospy.init_node('position_integration')
    rospy.Subscriber('/speed', Vector3, callback)
    global pub
    pub = rospy.Publisher('/position', Vector3, queue_size=10)
    rospy.spin()




if __name__ == '__main__':
    position_integration()