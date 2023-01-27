#!/usr/bin/env python3

# -*- coding : utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

rospy.init_node('vision')
   
bridge = CvBridge()        

def CV(msg):
    ######   np_arr = np.fromstring(msg.data, np.uint8)     #### fromstring을 더이상 못쓴다고 frombuffer를 사용하라고 함

    np_arr = np.frombuffer(msg.data, dtype = np.uint8) 
    img_BGR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)    
    img_HSV = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2HSV)
    img_GRAY = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2GRAY)
    _, img_binary1 = cv2.threshold(img_GRAY,127,255,cv2.THRESH_BINARY)  # 이진법을 통해서 임계값 기준으로 흑 백으로 나눔

    #### 카메라 포인트 따기
    red_color = (0,0,255)
    blue_color = (255,0,0)
    green_color = (0,255,0)
    violet_color  = (255,0,255)

    cam_size = [img_BGR.shape[1],img_BGR.shape[0]] ## 640,480
    hor_offset = 70
    ver_offset = 50
    point1 = (0,cam_size[1])                             # 좌하단
    point2 = (cam_size[0]//2 - hor_offset, cam_size[1]//2 + ver_offset) # 좌상단
    point3 = (cam_size[0]//2 + hor_offset, cam_size[1]//2 + ver_offset) # 우상단
    point4 = (cam_size[0],cam_size[1])                   # 우하단
    dst_point1 = (cam_size[0]//4,cam_size[1])
    dst_point2 = (cam_size[0]//4,0)
    dst_point3 = (int(cam_size[0]*(3/4)),0)
    dst_point4 = (int(cam_size[0]*(3/4)),cam_size[1]) 
    
    cv2.line(img_BGR,point1,point1,red_color,10)
    cv2.line(img_BGR,point2,point2,blue_color,10)
    cv2.line(img_BGR,point3,point3,green_color,10)
    cv2.line(img_BGR,point4,point4,violet_color,10)
    cv2.line(img_BGR,dst_point1,dst_point1,red_color,20)
    cv2.line(img_BGR,dst_point2,dst_point2,blue_color,20)
    cv2.line(img_BGR,dst_point3,dst_point3,green_color,20)
    cv2.line(img_BGR,dst_point4,dst_point4,violet_color,20)
    #print(point2)
   
    ####

    # kernel = np.ones((5,5),np.float32)/25
    # f2d = cv2.filter2D(img_BGR,-1,kernel)

    # ## gaussion blur
    # gau_img = cv2.GaussianBlur(img_binary1,(5,5),0)
    # cv2.imshow("gau_img",gau_img)
    # ## median blur
    # med_img = cv2.medianBlur(img_binary1,5)
    # cv2.imshow("med_img",med_img)
    # #bilateral blur
    # bil_img = cv2.bilateralFilter(img_binary1,-1,5,5)
    # cv2.imshow('bil_img',bil_img)

    
    #### 이미지 보기 ####
    cv2.imshow('cv_image_BGR',img_BGR)
    #cv2.imshow('cv_image_HSV',img_HSV)
    #cv2.imshow('cv_image_GARY',img_GRAY)
    #cv2.imshow('cv_image_BINARY',img_binary1)
    cv2.waitKey(1)

sub = rospy.Subscriber('/image_jpeg/compressed',CompressedImage,CV)



    

rospy.spin()