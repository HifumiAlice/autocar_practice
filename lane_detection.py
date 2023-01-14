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
    np_arr = np.fromstring(msg.data, np.uint8)     #### fromstring을 더이상 못쓴다고 frombuffer를 사용하라고 함
    np_arr = np.frombuffer(msg.data, dtype = np.uint8) 
    img_BGR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)    
    img_HSV = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2HSV)
    img_GRAY = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2GRAY)
    _, img_binary1 = cv2.threshold(img_GRAY,127,255,cv2.THRESH_BINARY)
    cv2.imshow('cv_image_BGR',img_BGR)
    cv2.imshow('cv_image_HSV',img_HSV)
    cv2.imshow('cv_image_GARY',img_GRAY)
    cv2.imshow('cv_image_BINARY',img_binary1)
    cv2.waitKey(1)

sub = rospy.Subscriber('/image_jpeg/compressed',CompressedImage,CV)



    

rospy.spin()