#! /usr/bin/env python3

#-*- coding:utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import time


degree = np.pi/180

class lidar():

    def __init__(self) :
        
        rospy.init_node("lidar_sub")
        ### 구독자, 발행자 선언
        self.pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
        self.pub_angle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)        
        sub_lidar2D = rospy.Subscriber('/lidar2D',LaserScan,callback = self.callback)

        #### 변수 설정
        self.roi_degree_offset = 60  ### 라이다 볼 인덱스 값
        self.infinity = float("inf")   ### inf는 뭐 정의가 되어있어 가능하다고 함
        self.previoustime = None  # 기다릴 시간 기준
        self.nexttime = None      # 
        self.timefind = False
        self.lencount = 0
        rospy.on_shutdown(self.lidar_shutdown)
        rospy.spin()

    def callback(self, data):

        #### 라이다 데이터 각 요소에 집어 넣기
        # angle_min = data.angle_min                  ### -3.1415927410125732 --> -pi == index 0이 가운데 <-- 정면을 지칭
        # angle_max = data.angle_max                  ### 3.1415927410125732 --> pi == index 0이 가운데 
        angle_increment = data.angle_increment      #0.01745329238474369 rad --> 0.9999999922536332 deg  몇도씩 증가할건가
        # time_incremet = data.time_increment         # 0.0001250000059371814 <-- ???
        # scan_time = data.scan_time                  # 0.04500000178813934  <-- ???
        range_min = data.range_min                  # 0.0  --> 아마도 m 단위 일거임
        range_max = data.range_max                  # 10.0 --> 아마도 m 단위 일거임
        # ranges = data.ranges                        # 360개의 1차원 배열(tuple로 들어옴) --> 0이 정면 값의 증가는 왼쪽 방향으로 증가 
        # intensities = data.intensities              ### 무언가의 배열?행렬?로 나옴 

        ## 변수 설정
        
        
        left_roi_lidar = list(data.ranges[0:self.roi_degree_offset + 1])          # 0 ~ 왼쪽 n번까지  +1 빼면 왼쪽 1도 덜 보는 거임
        right_roi_lidar = list(data.ranges[360 - self.roi_degree_offset : 360])   # 오른쪽 n번 ~ 359 까지
        roi_lidar = right_roi_lidar + left_roi_lidar  # 오른쪽n번 ~ 가운데 ~ 왼쪽 n번
        ## 0~n-1 : 오른쪽 값, n : 가운데값, n+1 ~ n*2 : 왼쪽 값
        right_roi_lidar = right_roi_lidar [ : : -1]

        for i in range(0,len(roi_lidar)):      #self.roi_degree_offset * 2+1) :
            if roi_lidar[i] >= 1.5 :
                roi_lidar[i] = self.infinity
            # if i < self.roi_degree_offset:                
            #     print(f"오른쪽 {self.roi_degree_offset-i:^3d}도 (index {self.roi_degree_offset+i + 240:^3d}) 거리 : {roi_lidar[i]} ")
            # elif i == self.roi_degree_offset:
            #     print(f"가운데       (index {0:^3d}) 거리 : {roi_lidar[i]}")
            # else:
            #     print(f"왼쪽   {i-self.roi_degree_offset:^3d}도 (index {i - self.roi_degree_offset :^3d}) 거리 : {roi_lidar[i]} ")
        #print(len(roi_lidar))
        # print(data.ranges[40]) 
        # print(roi_lidar[120]) 

        #### 동적 및 정적 판단
        speed = 1000
        angle = 0.5
        count = []
        
        for i in range(len(roi_lidar)):            
            if roi_lidar[i] != self.infinity:
                count.append(i)
                        
        if len(count) >= 4:                
            if self.timefind == False:
                self.previoustime = time.time()
                self.lencount = len(count)
                self.timefind = True

        if self.timefind == True:
            speed = 0
            self.nexttime = time.time()
            if self.nexttime - self.previoustime >= 4.0:
                if self.lencount - len(count) <= 4 :
                    print("정적")
                    angle = 0.0
                    speed = 1000
                else:
                    print("동적")
                    speed = 1000
                self.timefind = False
        
        print(self.lencount)
        
        print(f"길이 : {len(count)} 인덱스 : {count}")
        
        ### publish
        self.pub.publish(speed)
        self.pub_angle.publish(angle)
        ##### 동적 및 정적 판단 좀만 다듬으면 될듯

    def lidar_shutdown(self):
        print("Lidar is Dead !!")


if __name__ == "__main__":

    ld = lidar()