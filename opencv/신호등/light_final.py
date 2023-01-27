#! /usr/bin/env python3

# -*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np

from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

class Cam_light():

    def __init__(self):
        
        rospy.init_node("cam_light")
        self.SubCam = rospy.Subscriber("/image_jpeg/compressed",CompressedImage, self.callback) 
        self.PubCam1 = rospy.Publisher("PubCam_1",Image,queue_size = 3)
        self.PubCam2 = rospy.Publisher("PubCam_2",Image,queue_size = 3)  

        self.cv_bridge = CvBridge()  
        rospy.on_shutdown(self.CamShutdown)
        
        ### 변수 선언
        # 트랙바 ON/OFF
        self.Trackbar = False

        rospy.spin()
    
    

    

    def callback(self,data):
        
        ### 트랙바 켜기
        # if self.Trackbar == False:
        #     cv2.namedWindow("Trackbar",cv2.WINDOW_NORMAL)
        #     ## lower mask range
        #     cv2.createTrackbar("lower_h","Trackbar",90,359,nothing)
        #     cv2.createTrackbar("lower_s","Trackbar",100,255,nothing)
        #     cv2.createTrackbar("lower_v","Trackbar",80,255,nothing)

        #     ## upper mask range
        #     cv2.createTrackbar("upper_h","Trackbar",150,359,nothing)
        #     cv2.createTrackbar("upper_s","Trackbar",255,255,nothing)
        #     cv2.createTrackbar("upper_v","Trackbar",255,255,nothing)
            
        #     self.Trackbar = True

        # ##### 트랙바에서 값 받아오기
        # lower_h = cv2.getTrackbarPos("lower_h","Trackbar")
        # lower_s = cv2.getTrackbarPos("lower_s","Trackbar")
        # lower_v = cv2.getTrackbarPos("lower_v","Trackbar")
        # upper_h = cv2.getTrackbarPos("upper_h","Trackbar")
        # upper_s = cv2.getTrackbarPos("upper_s","Trackbar")
        # upper_v = cv2.getTrackbarPos("upper_v","Trackbar")

        # 토픽으로 image 파일 받아오기
        try:
            cv2_image = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Convert Error!")
            print(e)

        
        ### 이미지 변환하기
        
        image_hsv = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2HSV)
        image_hsv_roi = image_hsv[0:200, :] # 신호등 영역만 볼려고 
        image_roi = cv2_image[0:200, :]     # 신호등 영역만 볼려고
      
        ### 색깔 따기
        deg = 180/360
        
        # 빨간색 따기
        red_lower = np.array([deg * 0, 100, 80])  # deg * 0, 100,80
        red_upper = np.array([deg * 30, 255, 255])  # deg * 30, 255,255
        mask_red = cv2.inRange(image_hsv_roi,red_lower,red_upper)
        result_red = cv2.bitwise_and(image_roi,image_roi,mask = mask_red)
        red_color = round(result_red.mean(),3)
        
        # 초록색 따기
        green_lower = np.array([deg * 90, 100, 80])  # deg * 90, 100,80
        green_upper = np.array([deg * 150, 255,255])  # deg * 150, 255,255
        mask_green = cv2.inRange(image_hsv_roi,green_lower,green_upper)
        result_green = cv2.bitwise_and(image_roi,image_roi,mask = mask_green)       
        green_color = round(result_green.mean(),3)
        
        #### 신호 판별
        if red_color == 0 and green_color !=0:            ### 빨간색이 안잡히면 초록색만 나오므로 초록불
            print("초록불")
            #print("빨간색 :",red_color)
        elif green_color == 0 and red_color != 0:         ### 초록색만 안나오면 빨간불
            print("빨간불")
            #print("초록색 ",green_color)        
        elif red_color != 0 and green_color != 0:     ### 둘다 나오면 좌회전
            print("좌회전")
        else:
            print("주황불")
        
        print(f"초록색 {green_color}, 빨간색 {red_color}")
        ### 영상 보기 
        cv2.imshow('image_roi',image_roi)
        cv2.imshow('result_red',result_red) 
        cv2.imshow("result_green",result_green) 
        cv2.waitKey(1)

    def CamShutdown(self):
        print("Cam is Dead")
def nothing(x):
    pass
if __name__ == "__main__":
    Cam = Cam_light()