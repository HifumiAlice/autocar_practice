#! /usr/bin/env python3

# -*- coding:utf-8 -*-

# 허프변환을 통한 차선 찾기 y축만 로이를 적용함

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import Float64


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
        self.prev_flag = False
        rospy.spin()     
        

    
    def callback(self,data):

        ##### 트랙바 생성 ####
        if self.Trackbar == False:
            cv2.namedWindow("Trackbar",cv2.WINDOW_NORMAL)
            ## 이미지 변환 값 트랙바
            cv2.createTrackbar("CanThre1","Trackbar",100,255,nothing)
            cv2.createTrackbar("CanThre2","Trackbar",123,255,nothing)
            cv2.createTrackbar("blurThre1","Trackbar",10,50,nothing)
            #cv2.createTrackbar("blurThre1","Trackbar",5,100,nothing)

            ## 허프 라인 트랙바
            cv2.createTrackbar("Hough_Thre","Trackbar",15,150,nothing)
            cv2.createTrackbar("Hough_MinLen","Trackbar",15,150,nothing)
            cv2.createTrackbar("Hough_MinGap","Trackbar",10,150,nothing)
            
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

        # 잘린 화면에서 차선 딴거 원 화면에 그릴려고 변수 만듬
        roi_y = 450
        roi_x = 300
        image_Edgeroi = image_edge[400:720,:] #[400:720,:]
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

            if (slope < 0) and (x2 < 640):
                left_lines.append([Line.tolist()])

            elif (slope> 0 ) and (x1 > 640):
                right_lines.append([Line.tolist()])

        #### 좌우 차선 그리기 
        for line in left_lines:  #파란색 차선
            #line_img = cv2_image.copy()
            x1,y1, x2,y2 = line[0]
            cv2.line(image_original,(x1,y1+400),(x2,y2+400),(255,0,0),3)
        
        for line in right_lines: #빨간색 차선
            #line_img = cv2_image.copy()
            x1,y1, x2,y2 = line[0]
            cv2.line(image_original,(x1,y1+400),(x2,y2+400),(0,0,255),3)
        
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
            x1 = 0
            x2 = 0
        else:
            x1 = int((0.0 - b_left)/ m_left)
            x2 = int((320.0-b_left)/m_left)
        cv2.line(image_original,(x1,0+400),(x2,400+320),(0,255,0),2)

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
            x1 = 1280
            x2 = 1280
        else:
            x1 = int((0.0 - b_right)/ m_right)
            x2 = int((320.0-b_right)/m_right)
        cv2.line(image_original,(x1,0+400),(x2,400+320),(0,255,255),2)

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
        y_fix = int(720 - (320 - y_height))
        x_left = int(x_left)
        x_right = int(x_right)
        x_center = int(x_center)
        
        #print(y_fix)
        print(f"왼쪽 : {x_left}, 가운데 : {x_center}, 오른쪽 {x_right}")
        cv2.line(image_original,(0,y_fix),(1280,y_fix),(0,255,255),2)
        cv2.rectangle(image_original,(x_left-5,y_fix-5),(x_left+5,y_fix+5),(0,255,0),4)
        cv2.rectangle(image_original,(x_right-5,y_fix-5),(x_right+5,y_fix+5),(255,0,255),4)
        cv2.rectangle(image_original,(x_center-5,y_fix-5),(x_center+5,y_fix+5),(255,0,0),4)
        cv2.rectangle(image_original,(640-5,y_fix-5),(640+5,y_fix+5),(0,0,255),4)

        angle = x_center/1280
        print(angle)
        if angle < 0.5:
            speed = 1800*angle + 300
        else:
            speed = -1800 * angle +2100


        #### publish하기
        self.PubAngle.publish(angle)
        self.PubSpeed.publish(speed)        
        self.PubCam1.publish(self.cv_bridge.cv2_to_imgmsg(image_original,"rgb8"))


        ##### 이미지 보기 #####
        cv2.imshow("image_original",image_original)        
        #cv2.imshow("image_edge",image_edge)
        #cv2.imshow("image_blur",image_blur)
        cv2.imshow("image_roi",image_Edgeroi)
        #cv2.imshow("image_warp",image_warp)
        cv2.waitKey(1)


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



def nothing(x):
    pass

if __name__ == "__main__":
    Cam = Cam_lane()