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
from ultralytics import YOLO
from PIL import ImageEnhance
from PIL import Image as PIL_Image

height = 1080
width = 1920

def callback(data):
    # define picture to_down' coefficient of ratio
#    print('callback')
   
    
    cv_img = bridge.imgmsg_to_cv2(data, "bgra8")
#    print(cv_img.shape[0])
#    print("\n")
#    print(cv_img.shape[1])
#    print("\n")
    cv2.imshow("window" , cv_img)
#    print('show')
    cv2.waitKey(3)
    #resized_img = cv2.resize(cv_img, [1920,1080], interpolation=cv2.INTER_LINEAR)#
#    resized_img_rgb = cv2.COLOR_BGR2RGB(resized_img)
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
    #print(md_input.shape)
# Inference
    # results = model('/home/s/Desktop/catkin_workspace/src/gluon/image/image.jpg')
    
    # print(type(results[0]))
    #print(results.shape)
    #result_image = cv2.cvtColor(results.ims[0],cv2.COLOR_BGR2RGB)
    #cv2.imshow("result",results[0])
# Results
    #results.show()  # or .show(), .save(), .crop(), .pandas(), etc.
   

# API URL, use actual MODEL_ID
    # url = "/home/s/yolov8n.pt"

    # # Headers, use actual API_KEY
    # headers = {"x-api-key": "API_KEY"}

    # # Inference arguments (optional)
    # # data = {"size": 640, "confidence": 0.25, "iou": 0.45}

    # # Load image and send request
    # with open("/home/s/Desktop/catkin_workspace/src/gluon/image/image.jpg", "rb") as image_file:
    #     files = {"image": image_file}
    #     response = requests.post(url, headers=headers, files=files, data=data)

    # print(response.json())
    model = YOLO('/home/s/Desktop/catkin_workspace/src/gluon/model/best.pt')

    # Run inference
    results = model.predict('/home/s/Desktop/catkin_workspace/src/gluon/image/image.jpg',conf=0.4,classes=80)
    #results = model.predict('/home/s/Desktop/catkin_workspace/src/gluon/image/image.jpg')
    

    # Print image.jpg results in JSON format
    
    for r in results:
        #if(r.boxes)
        if(r.__len__()!=0):
            print(r.boxes.xywh)
            x = r.boxes.xywh[0,0]
            y = r.boxes.xywh[0,1]
            w = r.boxes.xywh[0,2]
            h = r.boxes.xywh[0,3]
            im_array = r.plot() # plot a BGR numpy array of predictions
            segment_img = cv_img[int(y-h/2)-20:int(y+h/2)+20,int(x-w/2)-20:int(x+w/2)+20]
            

            
            
            cv2.imshow("result",im_array)
            cv2.waitKey(3)
            cv2.imshow("input", segment_img)
            cv2.waitKey(3)
            im1 = PIL_Image.fromarray(cv2.cvtColor(segment_img,cv2.COLOR_BGR2RGB))
            #en=ImageEnhance.Brightness(pil_image)
            #im1=en.enhance(2)
            #en=ImageEnhance.Sharpness(im1)
            #im1=en.enhance(2)
            
            segment_img = cv2.cvtColor(np.asarray(im1),cv2.COLOR_RGB2BGR)
            segment_img = cv2.GaussianBlur(segment_img, (3, 3), 0)
            gray = cv2.cvtColor(segment_img, cv2.COLOR_BGR2GRAY)
            ret,img=cv2.threshold(segment_img,127,255,cv2.THRESH_BINARY)
            edge = cv2.Canny(img, 100, 100)

            se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
            binary = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, se) # 先膨胀再腐蚀
            contours, hireachy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            count = 0

            areas = []
            for c in range(len(contours)):
                areas.append(cv2.contourArea(contours[c]))

            #max_id1 = areas.index(max(areas))
            #areas.pop(max_id1)
            #max_id=areas.index(max(areas))
            max_id=areas.index(max(areas))
            max_rect = cv2.minAreaRect(contours[max_id])
            max_box = cv2.boxPoints(max_rect)
            max_box = np.int0(max_box)
            max_box[0,0]+=int(x-w/2)-20
            max_box[1,0]+=int(x-w/2)-20
            max_box[2,0]+=int(x-w/2)-20
            max_box[3,0]+=int(x-w/2)-20
            max_box[0,1]+=int(y-h/2)-20
            max_box[1,1]+=int(y-h/2)-20
            max_box[2,1]+=int(y-h/2)-20
            max_box[3,1]+=int(y-h/2)-20
            res = cv2.drawContours(cv_img, [max_box], 0, (255, 0, 0), 2)
                
                

            cv2.imshow("binary", binary)
            cv2.waitKey(3)
            cv2.imshow("draw_result", cv_img)
            cv2.waitKey(3)
            

       # a = r.boxes.cls.cpu().numpy()
        
        
        
            # for index in indexs:
        



 
def image_process_py():
    
    
 #   cv2.resizeWindow("window", 300, 300)
    rospy.init_node('image_process_py', anonymous=True)
    
    
    #global model
    # model = YOLO('yolov8n.pt')

    
    # make a video_object and init the video object
    global bridge
    
    bridge = CvBridge()
    
    rospy.Subscriber('/kinect2/hd/image_color', Image, callback)
    print('ready')
    
    
    rospy.spin()
 
if __name__ == '__main__':
    
    image_process_py()
