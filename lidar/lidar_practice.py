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
            if i < self.roi_degree_offset:                
                print(f"오른쪽 {self.roi_degree_offset-i:^3d}도 (index {self.roi_degree_offset+i + 240:^3d}) 거리 : {roi_lidar[i]} ")
            elif i == self.roi_degree_offset:
                print(f"가운데       (index {0:^3d}) 거리 : {roi_lidar[i]}")
            else:
                print(f"왼쪽   {i-self.roi_degree_offset:^3d}도 (index {i - self.roi_degree_offset :^3d}) 거리 : {roi_lidar[i]} ")
        #print(len(roi_lidar))
        # print(data.ranges[40]) 
        # print(roi_lidar[120]) 

        groupdata = self.grouping(roi_lidar)

        print(f"그룹된 갯수: {len(groupdata)}")
        for i in range(len(groupdata)):
            print("data : ",groupdata[i])
        # print("다음")

        #### 동적 및 정적 판단
        # speed = 1000
        # angle = 0.5
        # count = []
        
        # if len(groupdata) == 1:
        #     if self.timefind == False:
        #         self.previoustime = time.time()
        #         self.comparedata = groupdata[0]
        #         print("아아ㅏㅇ",self.comparedata)
        #         self.timefind = True
        #     speed = 0
        #     self.nexttime = time.time()
        #     if self.nexttime - self.previoustime >= 4.0:
        #         if abs(len(self.comparedata) - len(groupdata[0])) <= 2:
        #             print("정적")
        #             speed = 1000
        #             angle = 0.0
        #         else:
        #             print("동적")
        #             speed = 1000
        #         self.timefind = False
        
        

        
        ########## 원래꺼
        # for i in range(len(roi_lidar)):            
        #     if roi_lidar[i] != self.infinity:
        #         count.append(i)
                        
        # if len(count) >= 4:                
        #     if self.timefind == False:
        #         self.previoustime = time.time()
        #         self.lencount = len(count)
        #         self.timefind = True

        # if self.timefind == True:
        #     speed = 0
        #     self.nexttime = time.time()
        #     if self.nexttime - self.previoustime >= 4.0:
        #         if self.lencount - len(count) <= 4 :
        #             print("정적")
        #             angle = 0.0
        #             speed = 1000
        #         else:
        #             print("동적")
        #             speed = 1000
        #         self.timefind = False
        
        # print(f"길이 : {len(count)} 인덱스 : {count}")
        
        ### publish
        # self.pub.publish(speed)
        # self.pub_angle.publish(angle)
        ##### 동적 및 정적 판단 좀만 다듬으면 될듯

    def grouping(self,data):
        grouping_obs_flag = False
        oneObsIdx = []
        obs_pts = []

        for i in range(len(data)):
            # if i < len(data)//2:                
            #     print(f"오른쪽 {len(data)//2-i:^3d}도 (index {len(data)//2+i + 240:^3d}) 거리 : {data[i]} ") #240,320
            # elif i == len(data)//2:
            #     print(f"가운데       (index {0:^3d}) 거리 : {data[i]}")
            # else:
            #     print(f"왼쪽   {i-len(data)//2:^3d}도 (index {i - len(data)//2 :^3d}) 거리 : {data[i]} ") 

            if (data[i] == self.infinity) or (i == len(data)):  ### 마지막 각도에서 infiny 때문에 인식이 안됨
                    if grouping_obs_flag:
                        obs_pts.append(oneObsIdx)
                        grouping_obs_flag = False
                        continue

            if grouping_obs_flag == False:  ### grouping의 첫 데이터                
                if data[i] != self.infinity:
                    oneObsIdx = []
                    oneObsIdx.append(i)
                    obs_dis = data[i]
                    #print(objectsindex)
                    grouping_obs_flag = True
            else:  #grouping를 하고 있는 중
                ## 다른 물체로 판명되는 경우
                if abs(obs_dis - data[i]) > 0.10:
                    obs_pts.append(oneObsIdx)
                    oneObsIdx = []
                    oneObsIdx.append(i)
                else: # 같은 물체인 경우 
                    oneObsIdx.append(i)                    
                    #grouping_obs_flag = False                    
                obs_dis = data[i]
        
            
        
        return obs_pts
        
    def lidar_shutdown(self):
        print("Lidar is Dead !!")


if __name__ == "__main__":

    ld = lidar()