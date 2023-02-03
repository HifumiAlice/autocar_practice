#! /usr/bin/env python3

# -*- coding:utf-8 -*-

####### 허프변환을 통한 차선 구하기 roi를 사용했음 차선이 발견 안될 경우 화면 끝에 선 하나 그리는 중임

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import Float64, String
from math import *


class Cam_lane():

    def __init__(self):
        
        rospy.init_node("LaneFind")

        ### 구독자 발행자 선언
        self.SubCam = rospy.Subscriber("/image_jpeg/compressed",CompressedImage, self.callback)
        self.PubCam1 = rospy.Publisher("PubCam_1",Image,queue_size = 3)
        self.PubCam2 = rospy.Publisher("PubCam_2",Image,queue_size = 3)    
        self.PubSpeed = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
        self.PubAngle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)

        self.cv_bridge = CvBridge()

        ### 변수 선언
        # 트랙바 ON/OFF
        self.Trackbar = False

        # 차선 변수
        self.prev_r_mv = MovingAverage(15)
        self.prev_l_mv = MovingAverage(15)        
        self.mv = MovingAverage(5)
        self.prev_flag = False

        # 좌우 차선 선택
        # self.line_find = "left_line"
        #self.line_find = "right_line"        
        # self.line_find = "None"
        

        rospy.on_shutdown(self.CamShutdown)
        rospy.spin()     
        

    
    def callback(self,data):

        ##### 트랙바 생성 ####
        if self.Trackbar == False:
            cv2.namedWindow("Trackbar",cv2.WINDOW_NORMAL)
            cv2.namedWindow("line",cv2.WINDOW_NORMAL)
            ## 이미지 변환 값 트랙바
            cv2.createTrackbar("CanThre1","Trackbar",100,255,nothing)
            cv2.createTrackbar("CanThre2","Trackbar",123,255,nothing)
            cv2.createTrackbar("blurThre1","Trackbar",27,50,nothing)
            #cv2.createTrackbar("blurThre1","Trackbar",5,100,nothing)

            ## 허프 라인 트랙바
            cv2.createTrackbar("Hough_Thre","Trackbar",15,150,nothing)
            cv2.createTrackbar("Hough_MinLen","Trackbar",15,150,nothing)
            cv2.createTrackbar("Hough_MinGap","Trackbar",3,150,nothing)

            #### right(left) - N의 N값 설정
            cv2.createTrackbar("N","line",220,400,nothing)
            # linefide
            cv2.createTrackbar("linefind","line",0,2,nothing)
            
            self.Trackbar = True

        ##### 트랙바에서 값 받아오기
        CanThre1 = cv2.getTrackbarPos("CanThre1","Trackbar")
        CanThre2 = cv2.getTrackbarPos("CanThre2","Trackbar")
        blurThre1 = cv2.getTrackbarPos("blurThre1","Trackbar")
        if blurThre1 % 2 == 0:
            blurThre1 -= 1

        Hough_Thre = cv2.getTrackbarPos("Hough_Thre","Trackbar")
        Hough_MinLen = cv2.getTrackbarPos("Hough_MinLen","Trackbar")
        Hough_MinGap = cv2.getTrackbarPos("Hough_MinGap","Trackbar")
        #blurThre2 = cv2.getTrackbarPos("blurThre2","Trackbar")

        N = cv2.getTrackbarPos("N","line")
        self.line_find = cv2.getTrackbarPos("linefind","line")
        ##### 이미지 변환하기
        try:
            cv2_image = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Convertion Error!")
            print(e)

        height, width, channel = cv2_image.shape # 720,1280,3
        image_original = cv2_image.copy()

        image_hsv = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2HSV)
        image_gray = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2GRAY)


        ##### 일반적인 차선 따기
        image_blur = cv2.GaussianBlur(image_gray,(blurThre1,blurThre1),0)
        image_edge = cv2.Canny(np.uint8(image_blur),CanThre1,CanThre2)
        #image_Edgeroi = image_edge[int(height * 50/100):int(height * 100/100),int(width * 0/100): int(width * 100/100)]  # 세로, 가로

        #### 잘린 화면에서 차선 딴거 원 화면에 그릴려고 변수 만듬
        roi_y = 450 # 450 
        roi_x = 220 # 220
        image_Edgeroi = image_edge[roi_y:720,roi_x:1280-roi_x] #[400:720,:] 
        image_Edgeroi = cv2.Canny(np.uint8(image_Edgeroi),CanThre1,CanThre2)        
        #print(image_roi.shape)
        
        ######### 차선 라인 따기

        all_lines = cv2.HoughLinesP(image_Edgeroi,1,np.pi/180,Hough_Thre,Hough_MinLen,Hough_MinGap)  ## 1차로 주행 하려면 min_gap를 1로 해야 잘됨

        # for line in all_lines:
        #     #line_img = cv2_image.copy()
        #     x1,y1, x2,y2 = line[0]
        #     cv2.line(image_original,(x1,y1),(x2,y2),(0,255,0),3)

        #### 차선일 것 같은 기울기 골라내기
        slopes = []
        new_lines = []
        seta = []

        if type(all_lines) == type(None):  #### 라인 그릴거 없으면 그냥 넘기기
            pass
        else:
            #if all_lines.all:            
            for line in all_lines:            
                x1, y1, x2, y2 = line[0]
                if (x2-x1) == 0:
                    slope = 0
                    deg = 0
                else:
                    slope = float(y2-y1)/float(x2-x1)
                    rad = atan(slope) 
                    deg = rad * 180/pi ## <-- rad값을 deg 값으로 바꾸기
                    
                # 기울기가 너무 작은것은 차선에서 배제    상수는 타젠트 세타값임. 세타값을 구하려면 아크탄젠트 상수 python은 라디안으로 사용함, 라디안을 각도로 바꿔서 계산하거나 하면 될거같음
                # if 0.3333 < abs(slope) : # 상수값으로 차선 골라내기  #0.1은 세타 5.7도 쯤
                #     #print(slope)
                #     slopes.append(slope)
                #     new_lines.append(line[0])               
                #     seta.append(deg)
                
                if 22 <= abs(deg) :   ### 각도로 차선 골라내기
                    slopes.append(slope)
                    new_lines.append(line[0])               
                    seta.append(deg)
        
        ##### 좌우 차선 선분 찾기

        left_lines = []
        right_lines = []

        for j in range(len(slopes)):
            Line = new_lines[j]
            slope = slopes[j]

            x1,y1, x2,y2 = Line

            if (slope < 0) and (x2 < (1280-roi_x*2)/2):     ## 가운데 기준으로 왼쪽 차선 구하기
                left_lines.append([Line.tolist()])
                #print(f"왼쪽 {j}번쨰 기울기 {slope}")

            if (slope > 0 ) and (x1 > (1280-roi_x*2)/2):  ## 가운데 기준으로 오른쪽 차선 구하기
                right_lines.append([Line.tolist()])
                # print(f"오른쪽 {j}번쨰 기울기 {slope}")
                # print(f"오른쪽 {j}번쨰 각도 {seta[j]}")
            
            

        ### 좌우 차선 그리기 
        # 왼쪽 차선
        for line in left_lines:  #파란색 차선
            #line_img = cv2_image.copy()
    
            if all(line[0]):
                x1,y1, x2,y2 = line[0]            
                cv2.line(image_original,(x1+roi_x,y1+roi_y),(x2+roi_x,y2+roi_y),(255,0,0),3) ## 그릴거 있으면 이미지에 그리기
            # else:
            #     continue
            
        # 오른쪽 차선
        for line in right_lines: #빨간색 차선
            #line_img = cv2_image.copy()
            if all(line[0]):
                x1,y1, x2,y2 = line[0]
                cv2.line(image_original,(x1+roi_x,y1+roi_y),(x2+roi_x,y2+roi_y),(0,0,255),3) ## 그릴거 있으면 이미지에 그리기
            # else:
            #     continue
        
        ### 차선 중에서 대표 직선 구하기

        ## 왼쪽 대표 직선 구하기
        x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
        m_left, b_left = 0.0, 0.0
        size = len(left_lines)

        for line in left_lines:
            x1,y1, x2,y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        
        if size == 0:
            x_avg  = 0      # 없어도 될거같은데? 
            y_avg  = 0      # 없어도 될거같은데?
            m_left = 0      # 없어도 될거같은데?
            b_left = 0      # 없어도 될거같은데?
        else:
            x_avg = x_sum / (size * 2)
            y_avg = y_sum / (size * 2)
            m_left = m_sum / size
            b_left = y_avg-m_left * x_avg

        if size == 0: ## size가 0이라는 것은 라인을 못찾았다는 것이므로 화면 끝에다 그리기
            x1 = 0
            x2 = 0
            cv2.line(image_original,(x1,0+roi_y),(x2,roi_y+720-roi_y),(0,255,0),2)  
            
        else:      ## 0이 아니면 라인을 찾았다는 것이므로 라인에서 차선 그리기                
            x1 = int((0.0 - b_left)/ m_left)
            x2 = int((720-roi_y-b_left)/m_left)
            cv2.line(image_original,(x1+roi_x,0+roi_y),(x2+roi_x,roi_y+720-roi_y),(0,255,0),2)
        

        ### 오른쪽 대표 직선 구하기

        x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
        m_right, b_right = 0.0, 0.0
        size = len(right_lines)

        for line in right_lines:
            x1,y1, x2,y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        if size == 0:
            x_avg   = 0         # 없어도 될거같은데?
            y_avg   = 0         # 없어도 될거같은데?
            m_right = 0         # 없어도 될거같은데?
            b_right = 1280      # 없어도 될거같은데?
        else:
            x_avg  = x_sum / (size * 2)
            y_avg  = y_sum / (size * 2)
            m_right = m_sum / size
            b_right = y_avg-m_right * x_avg

        if size == 0:  #### roi width의 최대값을 넣어야함 
            x1 = 1280
            x2 = 1280
            cv2.line(image_original,(x1,0+roi_y),(x2,roi_y+720-roi_y),(0,255,255),2)
        else:            
            x1 = int((0.0 - b_right)/ m_right)
            x2 = int((720-roi_y-b_right)/m_right)
            cv2.line(image_original,(x1+roi_x,0+roi_y),(x2+roi_x,roi_y + 720-roi_y),(0,255,255),2)

        ### 좌우 차선의 좌표 구하기
        if self.prev_flag == False:
            self.prev_l_mv.add_sample(0)
            self.prev_r_mv.add_sample(0)
            self.prev_flat =True

        #140
        y_height = 140.0  # 사각형 그릴 때 로이랑 일반 카메라랑 위치 맞출려고 추가함   --> 로이 y축 위에서부터 아래로 내려감
        if m_left == 0.0:
            x_left = self.prev_l_mv.get_mm()
            
        else:
            x_left = int ((y_height-b_left)/m_left)  # y가 어딘가부터 720임 140에서 빼니깐 아래 사각형 그릴때도 참고 해야함
            #print((y_height-b_left)/m_left)
        
        if m_right == 0.0:
            x_right = self.prev_r_mv.get_mm()
        else:
            x_right = int((y_height - b_right)/m_right)
        
        self.prev_l_mv.add_sample(x_left)
        self.prev_r_mv.add_sample(x_right)
        

        
        #### 차선 각도 계산 사각형 그리기?
        y_fix = int(720 - ((720-roi_y) - y_height))
        cv2.line(image_original,(0,y_fix),(1280,y_fix),(255,55,255),2)     #### x축의 평행한 직선 그리기 여기축 기준에서 각도 잡을거임  (기준축이라고 하자)
        cv2.rectangle(image_original,(640-5,y_fix-5),(640+5,y_fix+5),(0,0,255),4) # 카메라의 센터 그리기, 기준축에 x좌표 센터를 그림ㅁ  # 원화면에 그린거라 맞음
        

        #### 차선 각도 계산 사각형 그리기?
        if m_left == 0.0: #x_left <= 0:
            x_left = 0
        else:
            x_left = int(x_left+roi_x)
            if x_left < 0 :
                x_left = 0

        if m_right == 0.0: #x_right <= 0:
            x_right = width
        else:
            x_right = int(x_right+roi_x)
            if x_right > width :
                x_right = width

        # if self.line_find == "left_line":
        #     x_center = x_left + N#350     # +- 상수는 로이x값보다 크면 안됨
        # elif self.line_find == "right_line":
        #     x_center = x_right  - N#300   # -250은 sector1에서 나가짐 그 이상은 안나가지는 것 같음 -270은 들어올때 들어와짐
        #     ## 우회전 심하게 하는 곳에서 MinGap = 1, xright-300(+-5)가 젤 적당한거 같다
        # else:
        #     x_center = (x_left + x_right) // 2

        if self.line_find == 1:
            x_center = x_left + N#350     # +- 상수는 로이x값보다 크면 안됨
        elif self.line_find == 2:
            x_center = x_right  - N#300   # -250은 sector1에서 나가짐 그 이상은 안나가지는 것 같음 -270은 들어올때 들어와짐
            ## 우회전 심하게 하는 곳에서 MinGap = 1, xright-300(+-5)가 젤 적당한거 같다
        else:
            x_center = (x_left + x_right) // 2
        
        #print(x_center)
        cv2.rectangle(image_original,(x_left-5,y_fix-5),(x_left+5,y_fix+5),(0,255,255),4)     #### 왼쪽 사각형 긋기
        cv2.rectangle(image_original,(x_right-5,y_fix-5),(x_right+5,y_fix+5),(255,255,0),4)   #### 오른쪽 사각형 긋기
        cv2.rectangle(image_original,(x_center-5,y_fix-5),(x_center+5,y_fix+5),(255,0,0),4)   #### 센터 사각형 긋기

        # print(f"왼쪽 : {x_left}, 가운데 : {x_center}, 오른쪽 {x_right}")

        angle = (x_center-roi_x)/((1280-roi_x*2))  ## 잘린 화면 기준으로 x가 840이 나옴 x_center가 화면의 가운데면 0.5가 나옴
        ## angle은 x센터 기준으로 잡았을 때 화면 왼쪽 끝에서 오른쪽 끝으로 1차 함수로 만들었음
        

        if  0<= angle < 0.5:
            speed = 1800 * angle + 300 # 1200
            #speed = 3400 * angle + 300  # 2000
            #speed = 5400 * angle + 300 # 3000
        elif angle <=1:
            speed = -1800 * angle + 2100
            #speed = 3400 * angle + 300
            #speed = -5400 * angle + 5700
        else:
            speed = 300

        
        # print("angle : ",angle)



        #### publish하기
        self.PubAngle.publish(angle)
        self.PubSpeed.publish(speed)        
        self.PubCam1.publish(self.cv_bridge.cv2_to_imgmsg(image_original,"rgb8"))
        #self.PubControl.publish()


        ##### 이미지 보기 #####
        cv2.imshow("image_original",image_original)        
        #cv2.imshow("image_edge",image_edge)
        #cv2.imshow("image_blur",image_blur)
        cv2.imshow("image_roi",image_Edgeroi)
        #cv2.imshow("image_warp",image_warp)
        cv2.waitKey(1)

    def CamShutdown(self):
        print("Cam is Dead!")
    
    


class MovingAverage ():

    def __init__(self,n):
        self.samples = n
        self.data = []
        self.weights = list (range(1, n + 1))
    
    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]
    
    def get_mm(self):
        return float(sum(self.data))/len(self.data)
    
    def get_wmm(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x* self.weights[i]
        
        return float(s) / sum(self.weights[:len(self.data)])

    

def nothing(x):
    pass

if __name__ == "__main__":
    Cam = Cam_lane()