#! /usr/bin/env python3

# -*- coding : utf-8 -*-

# 자율주행으로 정지선 인식, 정지 후 신호등 인식하기 초록불일 때 직진하는 코드 임시로 작성  # wego 시뮬레이터 22.R.3.2 버전

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64

class Cam_All():

    def __init__(self) :
        
        rospy.init_node("CamAll")

        ##### 발행자 구독자 선언
        self.SubCam = rospy.Subscriber("/image_jpeg/compressed",CompressedImage, self.callback)

        self.PubSpeed = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
        self.PubAngle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)
        self.PubCamUnder = rospy.Publisher("/pub_img_under", Image, queue_size=3)
        self.PubCamline = rospy.Publisher("/pub_img_line", Image, queue_size=3)
        self.PubCamLight1 = rospy.Publisher("/pub_img_light_red", Image, queue_size=3)
        self.PubCamLight2 = rospy.Publisher("/pub_img_light_green", Image, queue_size=3)

        self.cv_bridge = CvBridge()

        #####변수 선언
        self.width, self.height = 1280,720

        # 정지선 변수
        self.stop_mode = "None"
        self.srcpoint = np.array([[self.width * 10/100,self.height * 95/100],[self.width * 40/100 ,self.height * 60/100],[self.width * 60/100 ,self.height * 60/100],[self.width * 90/100 ,self.height * 95/100]])
        self.dstpoint = np.array([[self.width * 10/100 ,self.height * 100/100],[self.width * 10/100 ,self.height * 0/100],[self.width * 90/100 ,self.height * 0/100],[self.width * 90/100 ,self.height * 100/100]])
        # 신호등 변수
        self.light_mode = "None"
        self.light_find = "None"
        #차선 변수
        self.prev_r_mv = MovingAverage(15)
        self.prev_l_mv = MovingAverage(15)
        self.prev_flag = False
        
        self.Trackbar = False  # 트랙바

        self.angle = Float64()
        self.speed = Float64()

    

        rospy.on_shutdown(self.Cam_Shutdown)
        rospy.spin()

    def callback(self,data):

        ##### 트랙바 생성 ####
        if self.Trackbar == False:
            cv2.namedWindow("Trackbar",cv2.WINDOW_NORMAL)
            ## 이미지 변환 값 트랙바
            cv2.createTrackbar("CanThre1","Trackbar",100,255,nothing)
            cv2.createTrackbar("CanThre2","Trackbar",123,255,nothing)
            cv2.createTrackbar("blurThre1","Trackbar",27,50,nothing)
            #cv2.createTrackbar("blurThre1","Trackbar",5,100,nothing)

            ## 허프 라인 트랙바
            cv2.createTrackbar("Hough_Thre","Trackbar",15,150,nothing)
            cv2.createTrackbar("Hough_MinLen","Trackbar",15,150,nothing)
            cv2.createTrackbar("Hough_MinGap","Trackbar",3,150,nothing)
            
            self.Trackbar = True

        try:
            cv2_image = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Convertion Error!")
            print(e)

        
        self.lanefind(cv2_image)      # 차선 인식 함수

        # 정지선 인식
        if self.stop_mode == "None":
            self.stopline(cv2_image)      # 정지선 인식 함수
        elif self.stop_mode == "stopline":
            pass
        
        # 신호등 인식
       
        if self.light_mode == "None" and self.stop_mode == "stopline":
            #print("정지선 찾자")
            self.light(cv2_image)         # 신호등 인식 함수
        elif self.light_find == "green_light":
            pass
        
        # 각도 및 속도 설정
        if self.angle < 0.5:
            self.speed = 1400 * self.angle + 300
        else:
            self.speed = -1400 * self.angle +1700

        if self.stop_mode == "stopline":
            self.speed = 0
        
        if self.light_find == "green_light" :
            
            self.speed = 1000
            self.angle = 0.5
        print(self.light_find)


        

        #print(new_angle)

        #### publish하기
        self.PubAngle.publish(self.angle)
        self.PubSpeed.publish(self.speed)        
        
        # 이미지 퍼블리쉬
        self.PubCamUnder.publish(self.cv_bridge.cv2_to_imgmsg(self.image_roi))
        self.PubCamline.publish(self.cv_bridge.cv2_to_imgmsg(self.image_line,"rgb8"))
        self.PubCamLight1.publish(self.cv_bridge.cv2_to_imgmsg(self.result_red,"rgb8"))
        self.PubCamLight2.publish(self.cv_bridge.cv2_to_imgmsg(self.result_green,"rgb8"))
    
    # 정지선 인식
    def stopline(self,cv2_image):
        #print(self.srcpoint[3])
        src_under = np.float32([self.srcpoint[0],self.srcpoint[1],self.srcpoint[2],self.srcpoint[3]]) # 좌하단, 좌상단, 우상단, 우하단
        dst_under = np.float32([self.dstpoint[0],self.dstpoint[1],self.dstpoint[2],self.dstpoint[3]])        
        under_mask = cv2.getPerspectiveTransform(src_under,dst_under)
        wrap_image = cv2.warpPerspective(cv2_image,under_mask,(self.width,self.height)) 

        L= wrap_image[370:470,300:700] #60000 #[600:720,300:700]  # 픽셀 : 48000
        H,L,S = cv2.split(cv2.cvtColor(L,cv2.COLOR_BGR2HLS))
        _,self.L_roi = cv2.threshold(L,127,255,cv2.THRESH_BINARY)   ### 흰색 차선 찾기 좋음 즉 정지선 구분하기 좋다
        self.image_roi = self.L_roi 

        if cv2.countNonZero(self.image_roi) > 28000: #28000:   # 정지선이 인식이 되면 
            self.stop_mode = "stopline"
            self.stopline = "stopline"
            print(self.stop_mode)
        else:
            print("not stopline")

        #print(cv2.countNonZero(self.image_roi))
        # 픽셀값 구하는 코드 지워도 됨
        #print('이미지 전체 픽셀 개수 : {}'.format(self.image_roi.size))
        

    # 신호등 인식
    def light(self,cv2_image):
        image_hsv = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2HSV)
        image_hsv_roi = image_hsv[0:200, :] # 신호등 영역만 볼려고 
        image_roi = cv2_image[0:200, :]     # 신호등 영역만 볼려고
      
        ### 색깔 따기
        deg = 180/360
        
        #빨간색 따기
        red_lower = np.array([deg * 0, 100, 80])  # deg * 0, 100,80
        red_upper = np.array([deg * 30, 255, 255])  # deg * 30, 255,255
        mask_red = cv2.inRange(image_hsv_roi,red_lower,red_upper)
        result_red = cv2.bitwise_and(image_roi,image_roi,mask = mask_red)
        red_color = result_red.mean()
        red_color = round(red_color,3)
        self.result_red = result_red
        
        green_lower = np.array([deg * 90, 100, 80])  # deg * 90, 100,80
        green_upper = np.array([deg * 150, 255,255])  # deg * 150, 255,255
        mask_green = cv2.inRange(image_hsv_roi,green_lower,green_upper)
        result_green = cv2.bitwise_and(image_roi,image_roi,mask = mask_green)
       
        green_color = result_green.mean()
        green_color = round(green_color,3) 
        self.result_green = result_green

        if 0.085<= red_color <=0.215 :
            if 0.150<= green_color <= 0.270:
                print("좌회전")
                self.light_find = "left_go"
                #print("초록색 ",green_color)            
            else:
                print("빨간불")
                self.light_find = "red_light"
           # print("빨간색 :",red_color)

        if 0.500<= green_color <= 0.730:   ## 0.500 0.630
            #print("초록색 ",green_color)
            self.light_find = "green_light"
            print("초록불")

        print(f"초록색 {green_color}, 빨간색 {red_color}")

    #차선 인식 함수 허프 변환
    def lanefind(self,cv2_image):
        ##### 트랙바에서 값 받아오기
        CanThre1 = cv2.getTrackbarPos("CanThre1","Trackbar")
        CanThre2 = cv2.getTrackbarPos("CanThre2","Trackbar")
        blurThre1 = cv2.getTrackbarPos("blurThre1","Trackbar")
        if blurThre1 % 2 == 0:
            blurThre1 -= 1

        Hough_Thre = cv2.getTrackbarPos("Hough_Thre","Trackbar")
        Hough_MinLen = cv2.getTrackbarPos("Hough_MinLen","Trackbar")
        Hough_MinGap = cv2.getTrackbarPos("Hough_MinGap","Trackbar")
    
        #height, width, channel = cv2_image.shape # 720,1280,3
        image_original = cv2_image.copy()

        image_hsv = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2HSV)
        image_gray = cv2.cvtColor(cv2_image,cv2.COLOR_BGR2GRAY)


        ##### 일반적인 차선 따기
        image_blur = cv2.GaussianBlur(image_gray,(blurThre1,blurThre1),0)
        image_edge = cv2.Canny(np.uint8(image_blur),CanThre1,CanThre2)
        #image_Edgeroi = image_edge[int(height * 50/100):int(height * 100/100),int(width * 0/100): int(width * 100/100)]  # 세로, 가로

        # 잘린 화면에서 차선 딴거 원 화면에 그릴려고 변수 만듬
        roi_y = 450
        roi_x = 220
        image_Edgeroi = image_edge[roi_y:720,roi_x:1280-roi_x] #[400:720,:]
        image_Edgeroi = cv2.Canny(np.uint8(image_Edgeroi),CanThre1,CanThre2)        
        #print(image_roi.shape)
        
        ######### 차선 라인 따기

        all_lines = cv2.HoughLinesP(image_Edgeroi,1,np.pi/180,Hough_Thre,Hough_MinLen,Hough_MinGap)

        # for line in all_lines:
        #     #line_img = cv2_image.copy()
        #     x1,y1, x2,y2 = line[0]
        #     cv2.line(image_original,(x1,y1),(x2,y2),(0,255,0),3)

        #### 차선일 것 같은 기울기 골라내기
        slopes = []
        new_lines = []

        for line in all_lines:
            x1, y1, x2, y2 = line[0]

            if (x2-x1) == 0:
                slope = 0
            else:
                slope = float(y2-y1)/float(x2-x1)
            
            if 0.1 < abs(slope):
                slopes.append(slope)
                new_lines.append(line[0])        
        
        ##### 좌우 차선 선분 찾기

        left_lines = []
        right_lines = []

        for j in range(len(slopes)):
            Line = new_lines[j]
            slope = slopes[j]

            x1,y1, x2,y2 = Line

            if (slope < 0) and (x2 < (1280-roi_x)/2):
                left_lines.append([Line.tolist()])

            elif (slope > 0 ) and (x1 > (1280-roi_x)/2):
                right_lines.append([Line.tolist()])

        #### 좌우 차선 그리기 
        for line in left_lines:  #파란색 차선
            #line_img = cv2_image.copy()
            x1,y1, x2,y2 = line[0]
            cv2.line(image_original,(x1+roi_x,y1+roi_y),(x2+roi_x,y2+roi_y),(255,0,0),3)


        for line in right_lines: #빨간색 차선
            #line_img = cv2_image.copy()
            x1,y1, x2,y2 = line[0]
            cv2.line(image_original,(x1+roi_x,y1+roi_y),(x2+roi_x,y2+roi_y),(0,0,255),3)
        
        #### 차선 중에서 대표 직선 구하기

        ### 왼쪽 대표 직선 구하기

        x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
        m_left, b_left = 0.0, 0.0
        size = len(left_lines)

        for line in left_lines:
            x1,y1, x2,y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m_left = m_sum / size
        b_left = y_avg-m_left * x_avg

        if size == 0:
            x1 = roi_x
            x2 = roi_x
        else:
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

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m_right = m_sum / size
        b_right = y_avg-m_right * x_avg

        if size == 0:  #### roi width의 최대값을 넣어야함 
            x1 = 1280-roi_x
            x2 = 1280-roi_x
        else:
            x1 = int((0.0 - b_right)/ m_right)
            x2 = int((720-roi_y-b_right)/m_right)
        cv2.line(image_original,(x1+roi_x,0+roi_y),(x2+roi_x,roi_y + 720-roi_y),(0,255,255),2)

        ### 좌우 차선의 좌표 구하기
        if self.prev_flag == False:
            self.prev_l_mv.add_sample(0)
            self.prev_r_mv.add_sample(0)
            self.prev_flat =True


        y_height = 140.0  # 사각형 그릴 때 로이랑 일반 카메라랑 위치 맞출려고 추가함
        if m_left == 0.0:
            x_left = self.prev_l_mv.get_mm()
        else:
            x_left = int ((y_height-b_left)/m_left)  # y가 400부터 720임 140에서 빼니깐 아래 사각형 그릴때도 참고 해야함
        
        if m_right == 0.0:
            x_right = self.prev_r_mv.get_mm()
        else:
            x_right = int((y_height - b_right)/m_right)
        
        self.prev_l_mv.add_sample(x_left)
        self.prev_r_mv.add_sample(x_right)
        x_center = (x_left + x_right) // 2

        #### 차선 각도 계산 사각형 그리기?
        y_fix = int(720 - ((720-roi_y) - y_height))
        x_left = int(x_left+roi_x)
        x_right = int(x_right+roi_x)        
        x_center = int(x_center+roi_x)
        #print(x_center-640)
        
        #print(y_fix)
        #print(f"왼쪽 : {x_left}, 가운데 : {x_center}, 오른쪽 {x_right}")
        cv2.line(image_original,(0,y_fix),(1280,y_fix),(0,255,255),2)
        cv2.rectangle(image_original,(x_left-5,y_fix-5),(x_left+5,y_fix+5),(0,255,0),4)
        cv2.rectangle(image_original,(x_right-5,y_fix-5),(x_right+5,y_fix+5),(255,0,255),4)
        cv2.rectangle(image_original,(x_center-5,y_fix-5),(x_center+5,y_fix+5),(255,0,0),4)
        cv2.rectangle(image_original,(640-5,y_fix-5),(640+5,y_fix+5),(0,0,255),4)  # 원화면에 그린거라 맞음
        
        self.image_line = image_original
        self.angle = (x_center-roi_x)/((1280-roi_x*2))

    def Cam_Shutdown(self):
        print("Cam is Dead!")


def nothing(x):
    pass

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
    
if __name__ == "__main__":
    AllCam = Cam_All()