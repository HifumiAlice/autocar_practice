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
from ros_basics.msg import CamData,CamControl


class Cam_lane():

    def __init__(self):
        
        rospy.init_node("LaneFind")

        ### 구독자 발행자 선언
        self.SubCam = rospy.Subscriber("/image_jpeg/compressed",CompressedImage, self.callback)
        self.CamSub = rospy.Subscriber("/CamControl",CamControl,self.Cam_callback)
        self.CamPub = rospy.Publisher("/CamData",CamData, queue_size= 3)

        self.cv_bridge = CvBridge()

        ### 변수 선언
        self.height, self.width, self.channel = 720,1280,3
        # 트랙바 ON/OFF
        self.Trackbar = False
        # 차선 변수
        self.prev_r_mv = MovingAverage(15)
        self.prev_l_mv = MovingAverage(15)        
        self.mv = MovingAverage(5)
        self.prev_flag = False
        self.line_find = ""
        ## 정지선 변수
        self.srcpoint = np.array([[self.width * 10/100, self.height * 95/100],[self.width * 40/100 ,self.height * 60/100],[self.width * 60/100 ,self.height * 60/100],[self.width * 90/100 ,self.height * 95/100]])
        self.dstpoint = np.array([[self.width * 10/100, self.height * 100/100],[self.width * 10/100 ,self.height * 0/100],[self.width * 90/100 ,self.height * 0/100],[self.width * 90/100 ,self.height * 100/100]])
        ## On/Off 변수
        self.line_flag = True
        self.light_flag = True
        self.stopline_flag = True

        self.CamData = CamData()
        '''
        stopline : N,s,n --> None, Stop, nonstop
        light_find : N,r,g,l,n --> None, red, green, left, orange,
        line_find = N,l,r1,r2,a --> None, left, right1, right2, all
        '''

        rospy.on_shutdown(self.CamShutdown)
        rospy.spin()     
        

    
    def callback(self,data):

        ##### 이미지 변환하기
        try:
            cv2_image = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Convertion Error!")
            print(e)
        
        ### off가 됐으면 find는 None
        if self.line_flag: # 차선 찾기 필요시 on
            self.linefinde(cv2_image)
        else:              # 라바콘 및 회전로터리에서는 필요 없으니 off
            self.CamData.CamAngle = -1
            self.CamData.line_find = "N"
            
        if self.light_flag: # 교차로에서 필요 on
            self.light(cv2_image)
        else:               # 교차로 아니면 필요없음 off
            self.CamData.light_find = "N"

        if self.stopline_flag: # 교차로 및 회전 로터리, end지점에서 필요
            self.stopline(cv2_image)
        else:
            self.CamData.stopline_find = "N"

        cv2.waitKey(1)
        # print("stopline : ",self.CamData.stopline_find)
        # print("light : ", self.CamData.light_find)
        # print("linefind : ",self.CamData.line_find)
        # print("angle : ",self.CamData.CamAngle)
        # print(self.CamData)

        ## publish
        self.CamPub.publish(self.CamData)

    def Cam_callback(self,data):
        self.line_find = data.line_find
        self.line_flag = data.line_flag
        self.light_flag = data.light_flag
        self.stopline_flag = data.stopline_flag
        pass

    #### 신호등 함수
    def light(self,cv2_image):
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
        red_color = round(result_red.mean(),3)  ## 정확히는 모르겠지만 내가 지정한 색깔 범위에 인식이 되면 값이 나타남 mean을 통해 값을 확인
        
        # 초록색 따기
        green_lower = np.array([deg * 90, 100, 80])  # deg * 90, 100,80
        green_upper = np.array([deg * 150, 255,255])  # deg * 150, 255,255
        mask_green = cv2.inRange(image_hsv_roi,green_lower,green_upper)
        result_green = cv2.bitwise_and(image_roi,image_roi,mask = mask_green)       
        green_color = round(result_green.mean(),3)
        
        #### 신호 판별
        if red_color == 0 and green_color !=0:            ### 빨간색이 안잡히면 초록색만 나오므로 초록불
            #print("초록불")
            self.CamData.light_find = "g"
        elif green_color == 0 and red_color != 0:         ### 초록색만 안나오면 빨간불
            # print("빨간불")
            self.CamData.light_find = "r"
        elif red_color != 0 and green_color != 0:     ### 둘다 나오면 좌회전
            # print("좌회전")
            self.CamData.light_find = "l"
        else:
            # print("주황불")
            self.CamData.light_find = "n"
        
        # print(f"초록색 {green_color}, 빨간색 {red_color}")
        ### 영상 보기 
        cv2.imshow('image_light',image_roi)
        # cv2.imshow('result_red',result_red) 
        # cv2.imshow("result_green",result_green) 

    ### 정지선 함수
    def stopline(self,cv2_image):
        src_under = np.float32([self.srcpoint[0],self.srcpoint[1],self.srcpoint[2],self.srcpoint[3]]) # 좌하단, 좌상단, 우상단, 우하단  --> 따고 싶은 포인트를 저 순서대로 땃다는 것
        dst_under = np.float32([self.dstpoint[0],self.dstpoint[1],self.dstpoint[2],self.dstpoint[3]]) # 좌하단, 좌상단, 우상단, 우하단       
        under_mask = cv2.getPerspectiveTransform(src_under,dst_under)  #getPerspectiveTransform함수를 쓰려면 포인트?를 float32자료형으로 맞춰야 사용 가능
        wrap_image = cv2.warpPerspective(cv2_image,under_mask,(self.width,self.height)) # 이미지 중에서 포인터로 찝은 것을 원하는 사이즈로 만들어줌// 아마 내가 집은 영역보다 작게는 에러 뜰 듯?
        
        ## 정지선 인식
        L= wrap_image[620:720,300:700] #40000    #[600:720,300:700] 픽셀 : 48000   ### 차가 정지할 때 관성이 있음으로 차량 속도에 따라 로이 범위를 조절해 줘야함 y값만 조절 해주면 될거 같음
        H,L,S = cv2.split(cv2.cvtColor(L,cv2.COLOR_BGR2HLS))
        _,L_roi = cv2.threshold(L,127,255,cv2.THRESH_BINARY)   ### 흰색 차선 찾기 좋음 즉 정지선 구분하기 좋다 --> L 채널이 흰색을 찾기가 좋다는 뜻이다.
        
        if cv2.countNonZero(L_roi) > 17000:   ## 화면 내에 일정이상 흰색점이 발견됐다면 정지선이다.  전체 화면 픽셀수 대비 70% 이상이면 정지선으로 인식해도 될듯?   
            self.CamData.stopline_find = "s"
        else:
            # print("not stopline")
            self.CamData.stopline_find = "n"

        # print(cv2.countNonZero(L_roi)) # countnonzero 사용을 위해서 채널 한개인 이미지를 써야함 
        # 픽셀값 구하는 코드 지워도 됨
        #print('이미지 전체 픽셀 개수 : {}'.format(L_roi.size)) # 픽셀값을 모르겠다면 이걸로 알아볼 수 있다.
   
        ### 이미지 보기
        # cv2.imshow("cv2_image",cv2_image)
        # cv2.imshow("wrap_image",wrap_image)
        cv2.imshow("L_roi",L_roi)

    ### 차선 함수
    def linefinde(self,cv2_image):

        image_original = cv2_image.copy()
        image_gray = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2GRAY)

        ##### 일반적인 차선 따기
        image_blur = cv2.GaussianBlur(image_gray,(27,27),0)
        image_edge = cv2.Canny(np.uint8(image_blur),100,123)
        #image_Edgeroi = image_edge[int(height * 50/100):int(height * 100/100),int(width * 0/100): int(width * 100/100)]  # 세로, 가로

        #### 잘린 화면에서 차선 딴거 원 화면에 그릴려고 변수 만듬
        roi_y = 450 # 450 
        roi_x = 220 # 220
        image_Canny = image_edge[roi_y:720,roi_x:1280-roi_x] #[400:720,:] ## roi로 내가 원하는 차선만 확인
        
        ######### 차선 라인 따기
        all_lines = cv2.HoughLinesP(image_Canny,1,np.pi/180,15,15,1)  ## 1차로 주행 하려면 min_gap를 1로 해야 잘됨

        #### 차선일 것 같은 기울기 골라내기
        slopes = []
        new_lines = []
        seta = []

        if type(all_lines) == type(None):  #### 라인 그릴거 없으면 그냥 넘기기 --> 이 조건문 안쓰면 차선 안잡히는 곳에서는 카메라가 먹통됨
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
                
                if 22 <= abs(deg) :   ### 각도로 차선 골라내기 22도보다 크면 정지
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

            ## x축도 잘라서 차선을 구했기에 자른만큼 계산해줘야함
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
    
            if all(line[0]):  ### 그릴거 있으면 그려라
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
            x2 = int((720-roi_y-b_left)/m_left)  ## 720은 카메라 해상도의 따라서 바꿔줘야함 --> 1280 x 720 해상도 사용중임
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
            x_left = int ((y_height-b_left)/m_left)  # y가 위쪽 어딘가(roi해서)부터 720임 140에서 빼니깐 아래 사각형 그릴때도 참고 해야함
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
        cv2.rectangle(image_original,(640-5,y_fix-5),(640+5,y_fix+5),(0,0,255),4) # 카메라의 센터 그리기, 기준축에 x좌표 센터를 그림  # 원화면에 그린거라 맞음
        

        #### 차선 각도 계산 사각형 그리기?
        if m_left == 0.0: #x_left <= 0:
            x_left = 0
        else:
            x_left = int(x_left+roi_x)
            if x_left < 0 :
                x_left = 0

        if m_right == 0.0: #x_right <= 0:
            x_right = self.width
        else:
            x_right = int(x_right+roi_x)
            if x_right > self.width :
                x_right = self.width


        if self.line_find == "l":
            x_center = x_left + 320#350     # +- 상수는 로이x값보다 크면 안됨
            self.CamData.line_find = "l"
        elif self.line_find == "r1":
            x_center = x_right  - 220#300   # -250은 sector1에서 나가짐 그 이상은 안나가지는 것 같음 -270은 들어올때 들어와짐
            ## 우회전 심하게 하는 곳에서 MinGap = 1, xright-300(+-5)가 젤 적당한거 같다
            self.CamData.line_find = "r1"
        elif self.line_find == "r2":
            x_center = x_right - 280
            self.CamData.line_find = "r2"
        else:
            x_center = (x_left + x_right) // 2
            self.CamData.line_find = "a"
        
        
        #print(x_center)
        cv2.rectangle(image_original,(x_left-5,y_fix-5),(x_left+5,y_fix+5),(0,255,255),4)     #### 왼쪽 사각형 긋기
        cv2.rectangle(image_original,(x_right-5,y_fix-5),(x_right+5,y_fix+5),(255,255,0),4)   #### 오른쪽 사각형 긋기
        cv2.rectangle(image_original,(x_center-5,y_fix-5),(x_center+5,y_fix+5),(255,0,0),4)   #### 센터 사각형 긋기

        # print(f"왼쪽 : {x_left}, 가운데 : {x_center}, 오른쪽 {x_right}")

        self.CamData.CamAngle = (x_center-roi_x)/((1280-roi_x*2))  ## 잘린 화면 기준으로 x가 840이 나옴 x_center가 화면의 가운데면 0.5가 나옴
        ## angle은 x센터 기준으로 잡았을 때 화면 왼쪽 끝에서 오른쪽 끝으로 1차 함수로 만들었음
        

        # if  0<= self.CamData.CamAngle < 0.5: #각도에 따라서 속도 변환
        #     speed = 1800 * self.CamData.CamAngle + 300 # 1200
        #     #speed = 3400 * angle + 300  # 2000
        #     #speed = 5400 * angle + 300 # 3000
        # elif self.CamData.CamAngle <=1:
        #     speed = -1800 * self.CamData.CamAngle + 2100
        #     #speed = 3400 * angle + 300
        #     #speed = -5400 * angle + 5700
        # else:
        #     speed = 300

        # print("angle : ",self.CamData.CamAngle)
        # print("speed : ",speed)
        # print(f"find : {self.CamData.line_find}")

        # print("angle : ",angle)

        #### publish하기
        # self.PubAngle.publish(angle)
        # self.PubSpeed.publish(speed)        
        # self.PubCam1.publish(self.cv_bridge.cv2_to_imgmsg(image_original,"rgb8"))

        ##### 이미지 보기 #####
        cv2.imshow("image_original",image_original)        
        cv2.imshow("image_roi",image_Canny)

        
    def CamShutdown(self):
        print("Cam is Dead!")

class MovingAverage ():  ### 대표 차선구할때 사용

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