#! /usr/bin/env python3

# -*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64

import matplotlib.pyplot as plt

class StopLane():

    def __init__(self) :
        
        rospy.init_node("StopLane_find")
        self.sub = rospy.Subscriber('/image_jpeg/compressed',CompressedImage,callback = self.callback)
        self.pubcam1 = rospy.Publisher("/pub_img_under", Image, queue_size=10)
        self.pubcam2 = rospy.Publisher("/pub_img_under2", Image, queue_size=10)
        self.pub_speed = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
        
        self.stop_mode = 0
        self.cv_bridge = CvBridge()
        rospy.on_shutdown(self.CamShutdown)
        rospy.spin()
        pass

    def callback(self,data):
        try:
            cv2_image = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("convertion error")
            print(e)
        #np_arr = np.frombuffer(data.data, dtype = np.uint8) 
        #cv2_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        height , width, channel = cv2_image.shape
        ######## bird eye로 바꾸기
        src_under = np.float32(np.array([[width * 10/100,height * 95/100],[width * 40/100 ,height * 60/100],[width * 60/100 ,height * 60/100],[width * 90/100 ,height * 95/100]])) # 좌하단, 좌상단, 우상단, 우하단
        dst_under = np.float32(np.array([[width * 10/100 ,height * 100/100],[width * 10/100 ,height * 0/100],[width * 90/100 ,height * 0/100],[width * 90/100 ,height * 100/100]]))        
        under_mask = cv2.getPerspectiveTransform(src_under,dst_under)
        wrap_image = cv2.warpPerspective(cv2_image,under_mask,(width,height)) 

        # 정지선 인식
        if self.stop_mode == 0:
            self.detect_stopline(wrap_image,127)
        #elif self.stop_mode == 1:
        #self.detect_stopline(wrap_image,127)

        # under
        # try:
        #     self.pubcam1.publish(self.cv_bridge.cv2_to_imgmsg(self.image_roi))            
        # except CvBridgeError as e:
        #     print("publish error")
        #     print(e)
        # try:
        #     self.pubcam2.publish(self.cv_bridge.cv2_to_imgmsg(wrap_image,"rgb8"))            
        # except CvBridgeError as e:
        #     print("publish error")
        #     print(e)

        #cv2.waitKey(1)             


   
    def detect_stopline(self,frame,threshold_value): 
        L= frame[650:720,400:600]
        H,L,S = cv2.split(cv2.cvtColor(L,cv2.COLOR_BGR2HLS))
        _,self.L_roi = cv2.threshold(L,threshold_value,255,cv2.THRESH_BINARY)   ### 흰색 차선 찾기 좋음 즉 정지선 구분하기 좋다
        self.image_roi = self.L_roi#[500:620,400:600]  # 픽셀 : 14000 
        # cv2.imshow("frame",frame)
        # cv2.imshow("L",L)
        # cv2.imshow("self.image_roi",self.image_roi)

        if cv2.countNonZero(self.image_roi) > 10000:
            
            self.stop_mode = 1
            self.pub_speed.publish(0)
            print(cv2.countNonZero(self.image_roi))
        else:
            print("not stopline")
            self.pub_speed.publish(500)

        # 픽셀값 구하는 코드 지워도 됨
        self.img_show("Original", self.image_roi)
        print('이미지 전체 픽셀 개수 : {}'.format(self.image_roi.size))
        
    
   
    def CamShutdown(self):
        print("Cam is dead!")

    # 픽셀값 구하는 코드 지워도 됨
    def img_show(self,title='image', img=None, figsize=(8 ,5)):
        plt.figure(figsize=figsize)
    
        if type(img) == list:
            if type(title) == list:
                titles = title
            else:
                titles = []
    
                for i in range(len(img)):
                    titles.append(title)
    
            for i in range(len(img)):
                if len(img[i].shape) <= 2:
                    rgbImg = cv2.cvtColor(img[i], cv2.COLOR_GRAY2RGB)
                else:
                    rgbImg = cv2.cvtColor(img[i], cv2.COLOR_BGR2RGB)
    
                plt.subplot(1, len(img), i + 1), plt.imshow(rgbImg)
                plt.title(titles[i])
                plt.xticks([]), plt.yticks([])
    
            plt.show()
        else:
            if len(img.shape) < 3:
                rgbImg = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            else:
                rgbImg = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
            plt.imshow(rgbImg)
            plt.title(title)
            plt.xticks([]), plt.yticks([])
            plt.show()
    

if __name__ == "__main__":
    SL = StopLane()