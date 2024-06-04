#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import numpy as np
import torch
from torchvision import utils as vutils
from ultralytics import YOLO
from moveit import dual_moveitCommander
import message_filters
import tf2_ros
# 不要使用 geometry_msgs,需要使用 tf2 内置的消息类型
from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import tf
from PIL import ImageEnhance
from PIL import Image as PIL_Image
from std_msgs.msg import Int32


import threading

height = 1080
width = 1920


flag = 'y'
moveit_server = dual_moveitCommander.MoveIt_Control()
ready = 0
box_x = 0
box_y = 0
box_h = 0
box_w = 0
cv_img_depth = 0
left_end = 0
right_end = 0


def move_right_up():
    moveit_server.right_go_handup(is_wait=True)

def move_right_home():
    moveit_server.right_go_home(is_wait=True)
    


def move_left_up():
    moveit_server.left_go_handup(is_wait=True)


def move_left_home():
    moveit_server.left_go_home(is_wait=True)


def catch_callback(data1,data2):
    
    global flag
    global moveit_server
    global ready
    global box_h
    global box_w
    global box_x
    global box_y
    global cv_img_depth
    if(flag=='y'):
        cv_img = bridge.imgmsg_to_cv2(data1, "bgra8")
        cv_img_depth = bridge.imgmsg_to_cv2(data2, "32FC1")
        resized_img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        torch_img = torch.from_numpy(resized_img_rgb).to("cuda").div(255.0).unsqueeze(0)
        md_input = torch_img.permute(0, 3, 1, 2)

        assert (len(md_input.shape) == 4 and md_input.shape[0] == 1)
        # 复制一份
        md_input = md_input.clone().detach()
        # 到cpu
        md_input = md_input.to(torch.device('cpu'))
        # 反归一化
        # input_tensor = unnormalize(input_tensor)
        vutils.save_image(md_input, '/home/s/Desktop/catkin_workspace/src/gluon/image/image.jpg')
        
        model = YOLO('/home/s/Desktop/catkin_workspace/src/gluon/model/best.pt')

        # Run inference
        results = model.predict('/home/s/Desktop/catkin_workspace/src/gluon/image/image.jpg',conf = 0.4,classes=80)
        #results = model.predict('/home/s/Desktop/catkin_workspace/src/gluon/image/image.jpg')
        
        for r in results:
            if(r.__len__()!=0):
                flag = 'n'
                ready = 1
                print("go!")
                #print(r.boxes.xywh[0,0])
                im_array = r.plot() # plot a BGR numpy array of predictions



                box_x = r.boxes.xywh[0,0]
                box_y = r.boxes.xywh[0,1]
                box_w = r.boxes.xywh[0,2]
                box_h = r.boxes.xywh[0,3]
                # segment_img = cv_img[int(box_y-box_h/2)-20:int(box_y+box_h/2)+20,int(box_x-box_w/2)-20:int(box_x+box_w/2)+20]
                # pil_image = PIL_Image.fromarray(cv2.cvtColor(segment_img,cv2.COLOR_BGR2RGB))
                # en=ImageEnhance.Brightness(pil_image)
                # im1=en.enhance(2)
                # en=ImageEnhance.Sharpness(im1)
                # im1=en.enhance(2)
                # segment_img = cv2.cvtColor(np.asarray(im1),cv2.COLOR_RGB2BGR)
                # segment_img = cv2.GaussianBlur(segment_img, (3, 3), 0)
                # gray = cv2.cvtColor(segment_img, cv2.COLOR_BGR2GRAY)
                # edge = cv2.Canny(segment_img, 65, 88)

                # se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
                # binary = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, se) # 先膨胀再腐蚀
                # contours, hireachy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # count = 0

                # areas = []
                # for c in range(len(contours)):
                #     areas.append(cv2.contourArea(contours[c]))

                # max_id = areas.index(max(areas))
                # max_rect = cv2.minAreaRect(contours[max_id])
                # max_box = cv2.boxPoints(max_rect)
                # max_box = np.int0(max_box)
                # max_box[0,0]+=int(box_x-box_w/2)-20
                # max_box[1,0]+=int(box_x-box_w/2)-20
                # max_box[2,0]+=int(box_x-box_w/2)-20
                # max_box[3,0]+=int(box_x-box_w/2)-20
                # max_box[0,1]+=int(box_y-box_h/2)-20
                # max_box[1,1]+=int(box_y-box_h/2)-20
                # max_box[2,1]+=int(box_y-box_h/2)-20
                # max_box[3,1]+=int(box_y-box_h/2)-20
                # res = cv2.drawContours(cv_img, [max_box], 0, (255, 0, 0), 2)
                # segment_center = [int((max_box[0,1]+max_box[2,1])/2), int((max_box[0,0]+max_box[2,0])/2)]
                # print(segment_center)


                cv2.imshow("result",im_array)
                cv2.waitKey(1000)

                
                
                
def left_serial_callback(data):
    global left_end
    left_end = data.data
    # print(left_end)

def right_serial_callback(data):
    global right_end
    right_end = data.data
    # print(right_end)

 
def catch_demo():
    
    
 #   cv2.resizeWindow("window", 300, 300)
    rospy.init_node('catch_demo', anonymous=True)
    
    global pub
    pub = rospy.Publisher('can_state',JointState,queue_size=10)
    global left_pub
    left_pub = rospy.Publisher('left_joint_states',JointState,queue_size=10)
    global right_pub
    right_pub = rospy.Publisher('right_joint_states',JointState,queue_size=10)
    rospy.Subscriber('left_end_effector',Int32, left_serial_callback)
    rospy.Subscriber('right_end_effector',Int32, right_serial_callback)
    global can_state
    can_state = JointState()
    can_state.header = Header()
    
    can_state.name = ['cloud_platform']
    
    can_state.velocity = []
    can_state.effort = []

    global right_state
    right_state = JointState()
    right_state.header = Header()
    right_state.name = ['right_axis_joint_1','right_axis_joint_2','right_axis_joint_3','right_axis_joint_4','right_axis_joint_5','right_axis_joint_6']
    right_state.velocity = []
    right_state.effort = []
    global left_state
    left_state = JointState()
    left_state.header = Header()
    left_state.name = ['left_axis_joint_1','left_axis_joint_2','left_axis_joint_3','left_axis_joint_4','left_axis_joint_5','left_axis_joint_6']
    left_state.velocity = []
    left_state.effort = []
    #global model
    # model = YOLO('yolov8n.pt')

    
    # make a video_object and init the video object
    global bridge
    global ready
    global cv_img_depth
    global moveit_server
    
    bridge = CvBridge()
    
    t1= message_filters.Subscriber("/kinect2/hd/image_color", Image)
    t2 =message_filters.Subscriber("/kinect2/hd/image_depth_rect", Image)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1, allow_headerless=True)
    ts.registerCallback(catch_callback)
    print('ready')
    
    while 1:
        if ready==1:
                time.sleep(1)
                center = [int(box_y),int(box_x)]
                print(center)
                valid_point = []
                for i in range(-5,5):
                    for j in range(-5,5):
                        if(cv_img_depth[int(box_y+i)][int(box_x+j)]!=0):
                            valid_point.append([int(box_y+i),int(box_x+j)])
                depth_sum = 0
                for x in valid_point:
                    depth_sum = depth_sum + cv_img_depth[x[0]][x[1]]/1000.0
                depth = depth_sum/len(valid_point)
                rospy.loginfo("depth = %.2f",depth)
                center_point = [int(box_y),int(box_x),depth]
                center_posit = [(center_point[1]-9.5650919461068702e+02)*depth/1.2591516049668753e+03,(center_point[0]-5.3460791676518249e+02)*depth/1.2578749780456230e+03,depth]
                
                br = tf.TransformBroadcaster()
                br.sendTransform((center_posit[0],center_posit[1],center_posit[2]),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"catch_center","kinect2_link") 
                left_catch = [int(box_y-box_h/2),int(box_x),depth]
                right_catch = [int(box_y+box_h/2),int(box_x),depth]
                left_posit = [(left_catch[1]-9.5650919461068702e+02)*depth/1.2591516049668753e+03,(left_catch[0]-5.3460791676518249e+02)*depth/1.2578749780456230e+03,depth]
                right_posit = [(right_catch[1]-9.5650919461068702e+02)*depth/1.2591516049668753e+03,(right_catch[0]-5.3460791676518249e+02)*depth/1.2578749780456230e+03,depth]
                # print(left_posit)
                # print(right_posit)
                buffer = tf2_ros.Buffer()
                listener = tf2_ros.TransformListener(buffer)
                left_pose_stamped = PointStamped()
                left_pose_stamped.point.x = left_posit[0]
                left_pose_stamped.point.y = left_posit[1]
                left_pose_stamped.point.z = left_posit[2]
                left_pose_stamped.header.frame_id = "kinect2_link"
                center_pose_stamped = PointStamped()
                center_pose_stamped.point.x = center_posit[0]
                center_pose_stamped.point.y = center_posit[1]
                center_pose_stamped.point.z = center_posit[2]
                center_pose_stamped.header.frame_id = "kinect2_link"
            

                #left_point_target = buffer.transform(left_pose_stamped,"cloud_platform",rospy.Duration(5.0))
                # rospy.loginfo("转换结果:x = %.2f, y = %.2f, z = %.2f",
                #                 left_point_target.point.x,
                #                 left_point_target.point.y,
                #                 left_point_target.point.z)
                center_point_target = buffer.transform(center_pose_stamped,"cloud_platform",rospy.Duration(5.0))
                rospy.loginfo("转换结果:x = %.2f, y = %.2f, z = %.2f",
                                center_point_target.point.x,
                                center_point_target.point.y,
                                center_point_target.point.z)

                right_pose_stamped = PointStamped()
                right_pose_stamped.point.x = right_posit[0]
                right_pose_stamped.point.y = right_posit[1]
                right_pose_stamped.point.z = right_posit[2]



                right_pose_stamped.header.frame_id = "kinect2_link"
        
                #right_point_target = buffer.transform(right_pose_stamped,"cloud_platform",rospy.Duration(5.0))
                # rospy.loginfo("转换结果:x = %.2f, y = %.2f, z = %.2f",
                #                 right_point_target.point.x,
                #                 right_point_target.point.y,
                #                 right_point_target.point.z)
                #can_posit = np.arctan(((left_point_target.point.x+right_point_target.point.x)/2)/(-(left_point_target.point.y+right_point_target.point.y)/2))
                can_posit = np.arctan((center_point_target.point.x)/-(center_point_target.point.y))
                can_state.position = [np.pi/2+can_posit]
                print(can_state.position)


                moveit_server.left_go_handup(is_wait=True)
                moveit_server.right_go_handup(is_wait=True)


                # left_thread = threading.Thread(target=move_left_up)
                # right_thread = threading.Thread(target=move_right_up)
                # left_thread.start()
                # right_thread.start()
                # left_thread.join()
                # right_thread.join()


                # time.sleep(1)
                can_state.header.stamp = rospy.Time.now()
                pub.publish(can_state)
                time.sleep(8)
                center_point_target = buffer.transform(center_pose_stamped,"base_link",rospy.Duration(5.0))
                right_point_target = buffer.transform(right_pose_stamped,"base_link",rospy.Duration(5.0))
                left_point_target = buffer.transform(left_pose_stamped,"base_link",rospy.Duration(5.0))
                
                left_move = [center_point_target.point.x-0.13,center_point_target.point.y,center_point_target.point.z-0,np.pi/2,0,-np.pi/2]
                br.sendTransform((left_move[0],left_move[1],left_move[2]),tf.transformations.quaternion_from_euler(left_move[3],left_move[4],left_move[5]),rospy.Time.now(),"left_catch","base_link")
                moveit_server.left_move_position(left_move,is_wait=True)
                right_move = [center_point_target.point.x+0.30,center_point_target.point.y,center_point_target.point.z-0,-np.pi/2,0,-np.pi/2]
                br.sendTransform((right_move[0],right_move[1],right_move[2]),tf.transformations.quaternion_from_euler(right_move[3],right_move[4],right_move[5]),rospy.Time.now(),"right_catch","base_link")
                moveit_server.right_move_position(right_move,is_wait=True)


                # left_move[0] = left_point_target.point.x
                # right_move[0] = right_point_target.point.x
                # rospy.loginfo("left:x = %.2f, y = %.2f, z = %.2f",
                #                 left_move[0],
                #                 left_move[1],
                #                 left_move[2])
                # right_move[0] = right_point_target.point.x
                # rospy.loginfo("转换结果:x = %.2f, y = %.2f, z = %.2f",
                #                 left_move[0],
                #                 left_move[1],
                #                 left_move[2])
                # time.sleep(5)
                for i in range(10):
                    left_move[0] = center_point_target.point.x-0.13+i*0.013
                    right_move[0] = center_point_target.point.x+0.30-i*0.030
                    moveit_server.left_move_position(left_move,is_wait=True)
                    moveit_server.right_move_position(right_move,is_wait=True)
                    if left_end > 8:
                        break
                    if right_end > 8:
                        break
                # time.sleep(2)
                left_joint1 = moveit_server.robot.get_current_state().joint_state.position[1]
                right_joint1 = moveit_server.robot.get_current_state().joint_state.position[7]
                left_state.position = [0.0,0.0,0.0,0.0,0.0,0.0]
                right_state.position = [0.0,0.0,0.0,0.0,0.0,0.0]
                for i in range(6):
                        left_state.position[i] = moveit_server.robot.get_current_state().joint_state.position[i+1]
                        right_state.position[i] = moveit_server.robot.get_current_state().joint_state.position[i+7]
                for j in range(1,21):
                    
                    left_state.position[0] = left_state.position[0] - 0.02
                    right_state.position[0] = right_state.position[0] + 0.02
                    left_state.header.stamp = rospy.Time.now()
                    right_state.header.stamp = rospy.Time.now()
                    left_pub.publish(left_state)
                    right_pub.publish(right_state)
                    time.sleep(0.05)
                # for j in range(1,21):
                    
                #     left_state.position[0] = (-j)/10*left_joint1
                #     right_state.position[0] = (-j)/10*right_joint1
                #     left_state.header.stamp = rospy.Time.now()
                #     right_state.header.stamp = rospy.Time.now()
                #     left_pub.publish(left_state)
                #     right_pub.publish(right_state)
                #     time.sleep(0.05)
                can_state.position = [3*np.pi/2]
                can_state.header.stamp = rospy.Time.now()
                pub.publish(can_state)
                time.sleep(8)
                
                left_move[0] = center_point_target.point.x-0.25
                right_move[0] = center_point_target.point.x+0.25
                moveit_server.left_move_position(left_move,is_wait=True)
                moveit_server.right_move_position(right_move,is_wait=True)
                # time.sleep(2)
                moveit_server.left_go_home(is_wait=True)
                moveit_server.right_go_home(is_wait=True)
                ready = 0
                global flag
                flag = 'y'
    rospy.spin()
 
if __name__ == '__main__':
    
    catch_demo()


