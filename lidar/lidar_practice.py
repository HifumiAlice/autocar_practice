#! /usr/bin/env python3

#-*- coding:utf-8 -*-

import rospy
import numpy as np
import math
import time

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64




degree = np.pi/180

class lidar():

    def __init__(self) :
        
        rospy.init_node("lidar_sub")
        ### 구독자, 발행자 선언
        self.pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
        self.pub_angle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)        
        sub_lidar2D = rospy.Subscriber('/lidar2D',LaserScan,callback = self.callback)

        #### 변수 설정
        ### 라이다 값 처리 변수
        self.roi_degree_offset = 60  ### 라이다 볼 인덱스 값
        self.infinity = float("inf")   ### inf는 뭐 정의가 되어있어 가능하다고 함
        self.search_distance = 1.5
        self.Same_distance = 0.20
        self.obs_exist_flag = False

        ### 동적 정적 판단 변수
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
        angle_increment = data.angle_increment      # 0.01745329238474369 rad --> 0.9999999922536332 deg  몇도씩 증가할건가
        # time_incremet = data.time_increment         # 0.0001250000059371814 <-- ???
        # scan_time = data.scan_time                  # 0.04500000178813934  <-- ???
        range_min = data.range_min                  # 0.0  --> 아마도 m 단위 일거임
        range_max = data.range_max                  # 10.0 --> 아마도 m 단위 일거임
        # ranges = data.ranges                        # 360개의 1차원 배열(tuple로 들어옴) --> 0이 정면 값의 증가는 왼쪽 방향으로 증가 
        # intensities = data.intensities              ### 무언가의 배열?행렬?로 나옴 

        
        
        ################  거리만 집어 넣은 리스트
        # left_roi_lidar = list(data.ranges[0:self.roi_degree_offset + 1])          # 0 ~ 왼쪽 n번까지  +1 빼면 왼쪽 1도 덜 보는 거임
        # right_roi_lidar = list(data.ranges[360 - self.roi_degree_offset : 360])   # 오른쪽 n번 ~ 359 까지 #right_roi_lidar = right_roi_lidar [ : : -1]        
        # roi_lidar = right_roi_lidar + left_roi_lidar  # 오른쪽n번 ~ 가운데 ~ 왼쪽 n번  ## 0~n-1 : 오른쪽 값, n : 가운데값, n+1 ~ n*2 : 왼쪽 값
        
        # for i in range(0,len(roi_lidar)):      #self.roi_degree_offset * 2+1) :
        #     if roi_lidar[i] > 1.5 :
        #         roi_lidar[i] = self.infinity
        #     if i < self.roi_degree_offset:                
        #         print(f"오른쪽 {self.roi_degree_offset-i:^3d}도 (index {self.roi_degree_offset+i + 320:^3d}) 거리 : {roi_lidar[i]} ")

        #     elif i == self.roi_degree_offset:
        #         print(f"가운데       (index {0:^3d}) 거리 : {roi_lidar[i]}")
        #     else:
        #         print(f"왼쪽   {i-self.roi_degree_offset:^3d}도 (index {i - self.roi_degree_offset :^3d}) 거리 : {roi_lidar[i]} ")
        # #print(len(roi_lidar))
        # # print(data.ranges[40]) 
        # # print(roi_lidar[120]) 
        #########################


        ## 변수 설정
        obs_exist_flag = False
        count = 0
        left_roi_lidar = []
        right_roi_lidar = []
        for i in range(self.roi_degree_offset + 1):
            left_roi_lidar.append([i, data.angle_min + i * angle_increment, -i, data.ranges[i]]) ## origin index, theta, angle, distance # 가운데 기준으로 왼쪽은 -angle로 지정
        j = self.roi_degree_offset
        for i in range(360 - self.roi_degree_offset, 360):            
            right_roi_lidar.append([i, data.angle_min + i * angle_increment, j, data.ranges[i]]) ## origin index, theta, angle, distance    # 가운데 기준으로 오른쪽은 +angle로 지정
            j -= 1

        roi_lidar = right_roi_lidar + left_roi_lidar  ## origin index, theta, angle, distance
        
        for i in range(len(roi_lidar)):
            if roi_lidar[i][3] > 1.5 :
                roi_lidar[i][3] = self.infinity
            else:
                count += 1
            
            if count >= 1:
                obs_exist_flag = True
                
            print(f"{roi_lidar[i][2]}도의 index : {roi_lidar[i][0]}, 거리 : {roi_lidar[i][3]} ")
        #print("다음")

        
        if obs_exist_flag :
            groupdata = self.grouping(roi_lidar)  
            # print(groupdata[0])
            # print(roi_lidar[10])
            # print(f"그룹된 갯수: {len(groupdata)}")
            # for i in range(len(groupdata)):
            #     print(f"data : {groupdata[i]}, 거리 : {roi_lidar[groupdata[i][0]][3]} ")
                

            mgrpData = self.mergeObs(roi_lidar,groupdata)   ## roi index만 받아옴
            print(f"그룹된 갯수: {len(groupdata)}")
            print("최종 그룹 갯수:",len(mgrpData))
            print(mgrpData)
            print("다음")


            #동적 정적
            # if len(mgrpData):
            #     if self.timefind == False:
            #         self.previoustime = time.time()
            #         self.comparedata = mgrpData[0]
            #         print("아아ㅏㅇ",self.comparedata)
            #         self.timefind = True
            #     speed = 0
            #     self.nexttime = time.time()
            #     if self.nexttime - self.previoustime >= 1.0:
            #         print(f"비교할거 : {self.comparedata}, data : {mgrpData[0]}")
            #         if abs(len(self.comparedata) - len(mgrpData[0])) <= 2:
            #             print("정적")
            #             speed = 1000
            #             angle = 0.0
            #         else:
            #             print("동적")
            #             speed = 1000
            #         self.timefind = False
            #         self.previoustime = self.nexttime 
            
            # ### publish
            # self.pub.publish(speed)
            # self.pub_angle.publish(angle)

        ### 동적 및 정적 판단
        
        # speed = 1000
        # angle = 0.5
        # count = []
        
        # if len(mgrpData):
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
        ## data = [origin index, theta, angle, distance]
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
            #print(f"{data[i][2]:^2d}도의 index : {data[i][0]:^2d}, 거리 : {data[i][3]:^2f} ")
            if (data[i][3] == self.infinity):  ### 마지막 각도에서 infiny 때문에 인식이 안됨
                    if grouping_obs_flag:
                        obs_pts.append(oneObsIdx)
                        grouping_obs_flag = False
                        continue

            if grouping_obs_flag == False:  ### grouping의 첫 데이터                
                if data[i][3] != self.infinity:
                    oneObsIdx = []
                    oneObsIdx.append(i)
                    obs_dis = data[i][3]
                    #print(objectsindex)
                    grouping_obs_flag = True
            else:  #grouping를 하고 있는 중
                ## 다른 물체로 판명되는 경우
                if abs(obs_dis - data[i][3]) > self.Same_distance:
                    obs_pts.append(oneObsIdx)
                    oneObsIdx = []
                    oneObsIdx.append(i)
                else: # 같은 물체인 경우 
                    oneObsIdx.append(i)                    
                    #grouping_obs_flag = False                    
                obs_dis = data[i][3]
        
            if (i == len(data)-1):
                if grouping_obs_flag:
                        obs_pts.append(oneObsIdx)
                        grouping_obs_flag = False
                        continue
        
        return obs_pts  ## roi index만 있음 --> 그룹된 roi index
    
    def mergeObs(self, data, gdata) :  # 유효범위 데이터와 그룹핑된 오브젝트를 집어넣음
        mgrpData = []        
        #data  == [origin idx, theta, tmp_degree, _data.ranges[origin idx]]
        # gdata == [[물체가 찾아 졌을 때 i]* 개별로 판별된 수]
        #print(len(gdata))
        for i in range(len(gdata)) :
            tmpgdata = gdata[i]
            equalCount = False
            #print(f"data: {gdata[i]}")
            # print(f"temp1: {tmpgdata}")
            for j in range(i+1, len(gdata)) :
                tmp_g0 = data[gdata[i][-1]]  ## 그루핑 물체를 찾아낸 첫번째 i값을 집어넣는거
                tmp_g1 = data[gdata[j][0]]  ## 그루핑 물체를 찾아낸 첫번째 i값을 집어넣는거
                #print(f"go : {tmp_g0}, g1 : {tmp_g1}")
                idx1_xy = calc_axis_xy(tmp_g0[1], tmp_g0[3], 0, self.search_distance) # 세타, 인식된 거리, 최소거리, 물체를 찾기 위한 최대거리  --> x,y좌표를 반환 받음
                idx2_xy = calc_axis_xy(tmp_g1[1], tmp_g1[3], 0, self.search_distance) # 세타, 인식된 거리, 최소거리, 물체를 찾기 위한 최대거리  --> x,y좌표를 반환 받음
                distance = calc_distance(idx1_xy, idx2_xy) ## 각각의 물체끼리의 거리를 구함

                if abs(distance) > self.Same_distance : ## 구한 거리가 같은 물체의 임계값을 넘어가면 다른 물체가 맞고 아니면 같은 물체에 i값 넣기 / 거리 0.12m
                    #print("여기 들어오니?",distance)
                    continue
                # print("first : ",tmpgdata)
                tmpgdata = tmpgdata + gdata[j] 
                # print("second : ",tmpgdata) 
         
            for k in range(len(mgrpData)):
                tmp_pop0 = mgrpData[k].copy()
                tmp_pop1 = tmpgdata.copy()
                
                for tmp0 in tmp_pop0:
                    for tmp1 in tmp_pop1:
                        if tmp0 == tmp1 :
                            equalCount = True
                            # print(equalCount)
                            # print("같은거")

            #     # if len(tmp_pop0) - len(tmp_pop1) >= 0: 
            #     #     for q in range(len(tmp_pop1)):
            #     #         if (tmp_pop1.pop() == tmp_pop0.pop()) :  
            #     #             equalCount = True      
            #     #             pass
            #     #         else:
                            
            #     #             break
            #     # else:
            #     #     for q in range(len(tmp_pop0)):
            #     #         if (tmp_pop1.pop() == tmp_pop0.pop()) :    
            #     #             equalCount = True     
            #     #             pass
            #     #         else:                            
            #     #             break
            #     #### 둘의 길이가 다르다던가, pop된 요소만 다른 경우 다른 물체로 인식함

                # if tmp_pop0.pop() == tmp_pop1.pop():  ## 추가할 data가 "하나라도" 같은 data가 있다면 같은 물체 
                #     equalCount = True
                # else:
                #     continue

                

            # print(tmpgdata)
            if not equalCount :
                mgrpData.append(tmpgdata)  ## roi index --> 진짜로 다른 물체인지 판단한 roi index

        return mgrpData
        
    def lidar_shutdown(self):
        print("Lidar is Dead !!")


def calc_axis_xy(_theta, _distance, _min_range, _max_range) :
    if _min_range <= _distance <= _max_range :
        x = np.cos(_theta) * _distance
        y = np.sin(_theta) * _distance
        return [x, y]
    else :
        return [0, 0]
    
def calc_distance(pr, pl) :
    return math.sqrt(pow((pr[0] - pl[0]), 2) + pow((pr[1] - pl[1]), 2)) ### 삼각함수로 빗변 길이 구하기

if __name__ == "__main__":

    ld = lidar()