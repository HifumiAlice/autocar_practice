#!/usr/bin/env python3

# -*- coding : utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

rospy.init_node('vision')
   
bridge = CvBridge()     
### 변수 선언    
cam_size = [640,480]
Width, Height = 640, 480
mtx = np.array([ [422.037858, 0.0, 245.895397],
[0.0, 435.589734, 163.625535],
[0.0, 0.0, 1.0]])
dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

def CV(msg):
    ######   np_arr = np.fromstring(msg.data, np.uint8)     #### fromstring을 더이상 못쓴다고 frombuffer를 사용하라고 함

    np_arr = np.frombuffer(msg.data, dtype = np.uint8) 
    img_BGR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)    
    img_HSV = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2HSV)
    img_GRAY = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2GRAY)
    _, img_binary1 = cv2.threshold(img_GRAY,127,255,cv2.THRESH_BINARY)  # 이진법을 통해서 임계값 기준으로 흑 백으로 나눔

    ###### 색깔 지정
    red_color = (0,0,255)
    blue_color = (255,0,0)
    green_color = (0,255,0)
    violet_color  = (255,0,255)
    #### 카메라 포인트 따기
    #cam_size = [img_BGR.shape[1],img_BGR.shape[0]] ## 640,480
    hor_offset = 95
    ver_offset = 50

    src_point1 = (0,cam_size[1]-20)                             # 좌하단
    src_point2 = (cam_size[0]//2 - hor_offset, cam_size[1]//2 + ver_offset) # 좌상단
    src_point3 = (cam_size[0]//2 + hor_offset, cam_size[1]//2 + ver_offset) # 우상단
    src_point4 = (cam_size[0],cam_size[1]-20)                   # 우하단
    src_points = np.float32(np.array([src_point1,src_point2,src_point3,src_point4]))

    # dst_point1 = (cam_size[0]//4,cam_size[1])            # 좌하단
    # dst_point2 = (cam_size[0]//4,0)                      # 좌상단 
    # dst_point3 = (int(cam_size[0]*(3/4)),0)              # 우상단
    # dst_point4 = (int(cam_size[0]*(3/4)),cam_size[1])    # 우하단
    # dst_point1 = (0,cam_size[1])                           # 좌하단
    # dst_point2 = (0,0)                                     # 좌상단 
    # dst_point3 = (cam_size[0],0)                           # 우상단
    # dst_point4 = (cam_size[0],cam_size[1])                 # 우하단
    ## 점을 바꿀 위치 정함
    dst_point1 = (0,800)       
    dst_point2 = (0,0)     
    dst_point3 = (800,0)       
    dst_point4 = (800,800)     
    dst_points = np.float32(np.array([dst_point1,dst_point2,dst_point3,dst_point4]))
    
    ####### 차선 색깔 따기
    # 값 정하기
    color_deg = 255/360
    # 노랑선 따기
    yellow_lower = np.array([color_deg * 0,127,100])
    yellow_upper = np.array([color_deg * 60,255,255])
    yellow_mask = cv2.inRange(img_HSV,yellow_lower,yellow_upper)
    yellow_line_image = cv2.bitwise_and(img_HSV,img_HSV,mask=yellow_mask)
    #cv2.imshow("yello_lane",yellow_line_image)

    # 흰색 선 따기
    white_lower = np.array([color_deg*0,0,200])
    white_upper = np.array([color_deg*255,100,255])
    white_mask = cv2.inRange(img_HSV,white_lower,white_upper)
    white_line_image = cv2.bitwise_and(img_HSV,img_HSV,mask=white_mask)
    #cv2.imshow('white_lane',white_line_image)
    # 노랑색 하얀색 선 딴거 합치기
    yellow_white_line_image = cv2.bitwise_or(yellow_line_image,white_line_image)
    #cv2.imshow("yello_white",yellow_white_line_image)

    ######## 화면 옮기기 
    metrix = cv2.getPerspectiveTransform(src_points,dst_points) #float32의 자료형으로 받아야 함
    wrap_image = cv2.warpPerspective(img_BGR,metrix,(1000,900))         # 변환점으로 화면 옮김
    wrap_bgr_image = cv2.cvtColor(wrap_image,cv2.COLOR_HSV2BGR)
    warp_gray_image = cv2.cvtColor(wrap_bgr_image,cv2.COLOR_BGR2GRAY)
    _,warp_binary = cv2.threshold(warp_gray_image,100,255,cv2.THRESH_BINARY)
    #cv2.imshow("wrap_binary",warp_binary)

    
    ##### 선 그리기
    #cv2.line(img_BGR,src_point1,src_point1,red_color,10)
    #cv2.line(img_BGR,src_point2,src_point2,blue_color,10)
    #cv2.line(img_BGR,src_point3,src_point3,green_color,10)
    #cv2.line(img_BGR,src_point4,src_point4,violet_color,10)
     
    #cv2.line(img_BGR,dst_point1,dst_point1,red_color,20)
    #cv2.line(img_BGR,dst_point2,dst_point2,blue_color,20)
    #cv2.line(img_BGR,dst_point3,dst_point3,green_color,20)
    #cv2.line(img_BGR,dst_point4,dst_point4,violet_color,20)
    

    #cv2.line(img_BGR,src_point1,dst_point1,red_color,10)
    #cv2.line(img_BGR,src_point2,dst_point2,blue_color,10)
    #cv2.line(img_BGR,src_point3,dst_point3,green_color,10)
    #cv2.line(img_BGR,src_point4,dst_point4,violet_color,10)
   
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
    #cv2.imshow("wrap",wrap_image)
    cv2.waitKey(1)

    ######## 정지선 구하기
    height, width,_ = img_BGR.shape
    #print(img_BGR.shape)
    cvt_hls_img = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2HLS)
    h,l,s = cv2.split(255-cvt_hls_img)

    lower = (0,0,0)
    upper = (255,255,255)
    filterd_hls_img = cv2.inRange(cvt_hls_img,lower,upper)
    img_result = cv2.bitwise_and(img_BGR,img_BGR, mask = filterd_hls_img)
    
    # under
    zero_img = np.zeros_like(img_BGR)
    print(zero_img.shape,img_BGR.shape)
    region_under = np.array([[(0,height),(0,height*60/100),(width/2,height*40/100),(width,height*60/100),]],dtype=np.int32)
    region_mask_under = cv2.fillPoly(zero_img,region_under,(255,255,255))
    masked_under_img = cv2.bitwise_and(img_result,zero_img)
    #cv2.imshow('masked_under_img',masked_under_img)
    cv2.imshow('img_result',img_result)
    cv2.imshow('zero_img',zero_img)
    cv2.imshow("img_BGR",img_BGR)

    # under // warp
    src_under = np.float32([[width * 30/100,height], [width*70/100,height], [width*30/100,height*70/100] , [width*70/100,height*70/100]])# 좌하, 우하, 우상, 좌상
    dst_under = np.float32([[width * 15/100,height], [width*85/100,height], [width*15/100,height*0/100],   [width*85/100,height * 0/100]]) #좌하, 우하, 좌상, 우상
    #dst_under = np.float32([[width*0/100,height], [width*100/100,height], [width*0/100,0], [width*100/100,0]]) #좌하, 우하, 좌상, 우상
    M_under = cv2.getPerspectiveTransform(src_under,dst_under)
    warped_under_img = cv2.warpPerspective(zero_img,M_under,(width,height))   
    
    #print("height : {}, width : {}".format(height,width))
    cv2.imshow("warped_under_img",warped_under_img) 
    detect_stopline(warped_under_img,130)

def detect_stopline(cal_image, low_threshold_value):
        #stopline_roi, _, _ = self.set_roi(cal_image, 250, 350, 10)
        image = image_processing(cal_image, low_threshold_value)
        #image_roi = image[200:300,100:540]
        #cv2.imshow("roi",image_roi)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        #if cv2.countNonZero(image_roi) > 8000:
        #    print("stopline")
        #else:
        #    print("not stopline")

def image_processing(image, low_threshold_value):
    #self.blur = cv2.GaussianBlur(image, (5, 5), 0)
    _, L, _  = cv2.split(image)
    _, lane = cv2.threshold(L, low_threshold_value, 255, cv2.THRESH_BINARY)
    # cv2.imshow("L", self.L)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return lane


sub = rospy.Subscriber('/image_jpeg/compressed',CompressedImage,CV)



    

rospy.spin()