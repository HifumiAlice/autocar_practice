#! /usr/bin/env python3

# -*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
import math

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64

import matplotlib.pyplot as plt

class Cam_CV():

    def __init__(self):
        rospy.init_node("cam_node")

        self.pub_speed = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
        self.pub_angle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)
        self.sub = rospy.Subscriber('/image_jpeg/compressed',CompressedImage,callback=self.callback)
        self.cv_bridge = CvBridge()

        # self.angle = 17 # -19.5 ~ 19.5
        # self.angle = (self.angle + 19.5) / 39
        rospy.on_shutdown(self.CamShutdown)
        rospy.spin()
    

    def callback(self,data):
        try:
            cv2_image = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Convertion Error")
            print(e)
        
        image_hsv = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2HSV)
        image_gray = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2GRAY)
        
        height, width ,channel = cv2_image.shape  # 720,1280,3 

        #### 차선 색깔 따기
        color_deg = 255/360
        ###### 노란선 따기
        yellow_lower = np.array([color_deg * 0,127,100])
        yellow_upper = np.array([color_deg * 60,255,255])
        yellow_mask = cv2.inRange(image_hsv,yellow_lower,yellow_upper)
        yellow_line_image = cv2.bitwise_and(image_hsv,image_hsv,mask=yellow_mask)

        #### 흰색선 따기
        white_lower = np.array([color_deg*0,0,200])
        white_upper = np.array([color_deg*255,100,255])
        white_mask = cv2.inRange(image_hsv,white_lower,white_upper)
        white_line_image = cv2.bitwise_and(image_hsv,image_hsv,mask=white_mask)
       
        yellow_white_image = cv2.bitwise_or(yellow_line_image,white_line_image)
        
        ### 카메라 포인트 따기   width , height
        # 좌하단,좌상단,우상단,우하단 포인트 순
        #src_point1 = (int(width*5/100),int(height * 95/100))
        src_points = np.float32(np.array([[width * 0/100,height*95/100],[width * 35/100,height*55/100],[width * 65/100,height*55/100],[width * 100/100,height*95/100]])) 
        #src_points = np.float32(np.array([[width * 0/100,height*95/100],[width * 0/100,height*60/100],[width * 100/100,height*60/100],[width * 100/100,height*95/100]])) 
        dst_points = np.float32(np.array([(width * 0/100,height*100/100),(width * 0/100,height*0/100),(width * 100/100,height*0/100),(width * 100/100,height*100/100)]))
        
        
        ### 화면 bird eye로 바꾸기
        bird_mask = cv2.getPerspectiveTransform(src_points,dst_points)
        warp_image = cv2.warpPerspective(yellow_white_image,bird_mask,(width,height))
        warp_gray = cv2.cvtColor(warp_image,cv2.COLOR_BGR2GRAY)
        _,warp_binary =cv2.threshold(warp_gray,0,255,cv2.THRESH_BINARY) # 720,1280
       
        #print(warp_binary.shape)

        ###### 히스토그램 구하기
        # left, right, center, left_x, left_y, right_x, right_y , out_image= self.sliding_window(warp_binary)
        left, right, center, left_x, left_y, right_x, right_y, out_image,left_curverad, right_curverad = self.sliding_window(warp_binary)

        steering_rad = (math.atan(left_curverad)+math.atan(right_curverad))/2
        steering_degree = steering_rad * 180/np.pi
        steering = (-steering_degree+90) * 0.5

        if center[0][0] < 500:
            steering = -steering
            
        #### 화면 보기
        #cv2.line(cv2_image,src_point1,src_point1,(255,0,0),10)
        cv2.imshow("img",cv2_image)
        cv2.imshow("wrap_image",warp_image)
        # cv2.imshow("warp_binary",warp_binary)  
        cv2.imshow("out_image",out_image)  
        cv2.waitKey(1)

        ###### publishing
        angle = steering # -19.5 ~ 19.5
        angle = (angle + 19.5) / 39
        if angle >= 0.5:
            speed = -2400*angle + 2700 
        else:
           speed = 2400*angle + 300
        
        if speed < 0 :
            speed = 300      ########## 각도가 너무 튐

        print("steering: ",steering)
        print("속도 : ",speed)
        print("앵글 : ",angle)
        self.pub_angle.publish(angle)
        self.pub_speed.publish(speed)

    def sliding_window(self,warp_binary):
        ##### y축 기준 절반 아래 부분만을 사용하여 x축 기준 픽셀의 분포를 구함
        half_y = warp_binary.shape[0]/2
        histogram = np.sum(warp_binary[int(half_y):,:],axis=0) 
        # 히스토그램을 절반으로 나누어 좌우 히스토그램의 최대값의 인덱스 반환    
        midpoint = np.int(histogram.shape[0]/2)
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint
        #print(left_base)
        
        left_current = left_base        
        right_current = right_base
        

        warp_binary = warp_binary *255        
        out_image = np.dstack((warp_binary,warp_binary,warp_binary)) * 255
        ### window
        ## 적절한 윈도우의 개수를 지정한다
        # 개수가 너무 적으면 정확하게 차선을 찾기가 힘들고 너무 많으면 연산량 증가로 시간이 오래 걸린다
        windows = 9
        window_height = np.int(warp_binary.shape[0] / windows) 
        #윈도우의 넓이를 지정한다, 윈도우가 옆 차선까지 넘어가지 않게 사이즈를 적절히 지정한다        
        margin = 80
        # 탐색할 최소 픽셀의 개수를 지정한다
        minpix = 30

        lanepixel = warp_binary.nonzero()
        lanepixel_y = np.array(lanepixel[0])
        lanepixel_x = np.array(lanepixel[1])

        # pixel index 담을 list
        left_lane_idx = []
        right_lane_idx = []

        # step through the windows one by one
        for window in range(windows):
            
            # window boundary 지정 (가로)            
            window_y_low = warp_binary.shape[0] - (window + 1) * window_height
            window_y_high = warp_binary.shape[0] - window * window_height
            # print("{}     {}".format(window_y_low,window_y_high))
            # position 기준 window size
            window_x_left_low = left_current-margin
            window_x_left_high = left_current+margin
            window_x_right_low = right_current-margin
            window_x_right_high = right_current+margin

            #window 시각화
            cv2.rectangle(out_image,(window_x_left_low, window_y_low),(window_x_left_high,window_y_high),(0,0,255),2)
            cv2.rectangle(out_image,(window_x_right_low,window_y_low),(window_x_right_high,window_y_high),(255,0,0),2)


            # 왼쪽 오른쪽 각 차선 픽셀이 window안에 있는 경우 index 저장
            left_idx =  ((lanepixel_y >= window_y_low) & (lanepixel_y < window_y_high)&(lanepixel_x >= window_x_left_low)&(lanepixel_x < window_x_left_high)).nonzero()[0]
            right_idx = ((lanepixel_y >= window_y_low) & (lanepixel_y < window_y_high)&(lanepixel_x >= window_x_right_low)&(lanepixel_x < window_x_right_high)).nonzero()[0]
            
            # append these indices to the lists
            left_lane_idx.append(left_idx)
            right_lane_idx.append(right_idx)
            
 

            # window내 설정한 pixel개수 이상이 탐지 되면, 픽셀들의 x 좌표 평균으로 업데이트
            if len(left_idx) > minpix:
                left_current = np.int(np.mean(lanepixel_x[left_idx]))
            if len(right_idx) > minpix:
                right_current = np.int(np.mean(lanepixel_x[right_idx]))


        # np.concatenate(array) => axis 0으로 차원 감소(window개수로 감소)
        left_lane_idx = np.concatenate(left_lane_idx)    
        right_lane_idx = np.concatenate(right_lane_idx)  

        
        # window 별 좌우 도로 픽셀 좌표  
        left_x = lanepixel_x[left_lane_idx]
        left_y = lanepixel_y[left_lane_idx]
        right_x = lanepixel_x[right_lane_idx]
        right_y = lanepixel_y[right_lane_idx]        
        
        # 좌우 차선 별 2차 함수 계수 추정
        leftfit = np.polyfit(left_y,left_x,2)
        rightfit = np.polyfit(right_y,right_x,2)
        
        # 좌우 차선별 추정할 y좌표
        ploty = np.linspace(0,out_image.shape[0]-1,3)

        # 좌우 차선 별 2차 곡선 추정
        left_fit_x = leftfit[0]* ploty**2 + leftfit[1] * ploty+leftfit[2]
        right_fit_x = rightfit[0]*ploty**2 + rightfit[1] * ploty+rightfit[2]
        center_fit = (left_fit_x + right_fit_x ) / 2

        # window 안의 lane을 black 처리
        # out_image[left_y,left_x] = (0,0,0)
        # out_image[right_y,right_x] = (0,0,0)

        # 양쪽 차선 및 중심 선 pixel 좌표 (x,y)로 변환
        left = np.asarray(tuple(zip(left_fit_x,ploty)),np.int32)
        center = np.asarray(tuple(zip(center_fit,ploty)),np.int32)
        right = np.asarray(tuple(zip(right_fit_x,ploty)),np.int32)

        cv2.polylines(out_image,[left],False,(255,0,0),thickness=5)
        cv2.polylines(out_image,[right],False,(0,0,255),thickness=5)
        cv2.polylines(out_image,[center],False,(255,0,255),thickness=5)

        ############ 곡률? 뭔 계산 했다고 함
        ### 임의의 설정 값
        xm_per_pix = 0.001
        ym_per_pix = 0.001 
        y_eval = 480

        left_fit_cr = np.polyfit(left_y*ym_per_pix, left_x*xm_per_pix,2)
        right_fit_cr = np.polyfit(right_y*ym_per_pix, right_x*xm_per_pix,2)

        left_curverad = ((1+(2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5)/np.absolute(2*left_fit_cr[0])
        right_curverad = ((1+(2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5)/np.absolute(2*left_fit_cr[0])
        return left, right, center, left_x, left_y, right_x, right_y, out_image, left_curverad, right_curverad

        
        


    def CamShutdown(self):
        print("Cam is Dead")

if __name__ == "__main__":
    cam = Cam_CV()