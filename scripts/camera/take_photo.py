#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
import torch
from torchvision import utils as vutils
import threading
import message_filters
from pyzbar import pyzbar

height = 1080
width = 1920

flag = 'q'
count = 1

class myThread (threading.Thread):   #继承父类threading.Thread
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):                   #把要执行的代码写到run函数里面 线程在创建后会直接运行run函数 
        global flag
        while(1):
            flag = input('').split(" ")[0]
        
        



def callback(data1,data2):
    # define picture to_down' coefficient of ratio
#    print('callback')
   
    global flag
    global count

    cv_img = bridge.imgmsg_to_cv2(data1, "bgra8")
    cv_img_depth = bridge.imgmsg_to_cv2(data2, "32FC1")
    
#    print(cv_img.shape[0])
#    print("\n")
#    print(cv_img.shape[1])
#    print("\n")
    cv2.imshow("photoes" , cv_img)
#    print('show')
    cv2.waitKey(100)
    cv2.imshow("depth_photoes" , cv_img_depth)
#    print('show')
    cv2.waitKey(100)
    
    if(flag=='t'):
        print("take")

        cv2.imwrite('/home/s/Desktop/catkin_workspace/src/gluon/calibrate/color_image'+str(count)+'.png',cv_img)
        # resized_img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        # torch_img = torch.from_numpy(resized_img_rgb).to("cuda").div(255.0).unsqueeze(0)
        # photo = torch_img.permute(0, 3, 1, 2)

        # assert (len(photo.shape) == 4 and photo.shape[0] == 1)
        # cv2.imwrite('/home/s/Desktop/catkin_workspace/src/gluon/calibrate/color_image'+str(count)+'.png',cv_img)
        # count = count + 1
        # # 到cpu
        # photo = photo.to(torch.device('cpu'))
        # # 反归一化
        # # input_tensor = unnormalize(input_tensor)
        # vutils.save_image(photo, '/home/s/Desktop/catkin_workspace/src/gluon/image/color_image'+str(count)+'.jpg')
        # gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        # ret, thresh = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY)
        # # 实例化
        # barcodes = pyzbar.decode(thresh)
        # if(len(barcodes)==1):
        #     cv2.imwrite('/home/s/Desktop/catkin_workspace/src/gluon/calibrate/color_image'+str(count)+'.png',cv_img)
        #     print(barcodes[0].data.decode('ascii'))
        #     # def decode(image, barcodes):
        #     #     # 循环检测到的条形码
        #     for barcode in barcodes:
        #         # 提取条形码的边界框的位置
        #             # 画出图像中条形码的边界框
        #         (x, y, w, h) = barcode.rect
        #         cv2.rectangle(cv_img, (x, y), (x + w, y + h), (255, 0, 0), 5)
        #         # cv2.imshow("result",cv_img)
        #         # cv2.waitKey(0)
            
        #     print(cv_img_depth[int(y+h/2)][int(x+w/2)])
        #     cv2.imwrite('/home/s/Desktop/catkin_workspace/src/gluon/calibrate/qr_image'+str(count)+'.png',cv_img)
            
        #     cv2.imwrite('/home/s/Desktop/catkin_workspace/src/gluon/calibrate/depth_image'+str(count)+'.png',cv_img_depth.astype(np.uint16))

        
        count = count + 1
        flag = 'q'

def take_photo():
    
    
 #   cv2.resizeWindow("window", 300, 300)
    rospy.init_node('take_photo', anonymous=True)
    
    
    # make a video_object and init the video object
    global bridge
    
    
    bridge = CvBridge()
    
    
    t1= message_filters.Subscriber("/kinect2/hd/image_color", Image)
    t2 =message_filters.Subscriber("/kinect2/hd/image_depth_rect", Image)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)

    take_it = myThread(1,"take_it",1)
    take_it.start()
    print('ready')
    
    
    rospy.spin()


if __name__ == '__main__':
    take_photo()