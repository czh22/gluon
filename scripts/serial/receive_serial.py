#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8

# serial_rxd.py
import time
import serial
import rospy
from std_msgs.msg import Int32

rospy.init_node('receive_serial', anonymous=True)
left_pub = rospy.Publisher('left_end_effector',Int32,queue_size=10)
right_pub = rospy.Publisher('right_end_effector',Int32,queue_size=10)
left_msg = Int32()
right_msg = Int32()

# set serial port initialized parameters
com = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 9600,
    bytesize = serial.EIGHTBITS,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
)

# wait 1s for serial port initialization
time.sleep(1)

# received data and print in hex string form
while 1:
    rxd_num = com.inWaiting()
    if rxd_num > 0:
        rxd = com.read(rxd_num)
        
        
        if rxd[0] == 49:
            right_msg.data = int.from_bytes(rxd[1:],byteorder='little',signed=False)
            print(right_msg.data)
            right_pub.publish(right_msg)
        elif rxd[0] == 50:
            left_msg.data = int.from_bytes(rxd[1:],byteorder='little',signed=False)
            print(left_msg.data)
            left_pub.publish(left_msg)
