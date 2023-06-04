#!/usr/bin/env python3

# -*- coding:utf-8 -*-

import rospy
import numpy as np
import time
import math

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from ros_basics.msg import LidarData, LidarControl


class lidar():

    def __init__(self) :
        
        rospy.init_node("lidar_sub")
        ### 구독자, 발행자 선언
        self.pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
        self.pub_angle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)        
        sub_lidar2D = rospy.Subscriber('/lidar2D',LaserScan,callback = self.callback)

        self.LidarPub = rospy.Publisher("/LidarData",LidarData,queue_size=3)
        self.LidarControl = rospy.Subscriber("/LidarControl",LidarControl,self.LidarCallback)

        #### 변수 설정
        ### 라이다 값 처리 변수
        self.roi_degree_offset = 20  ### 라이다 볼 인덱스 값
        self.infinity = float("inf") ### inf는 뭐 정의가 되어있어 가능하다고 함
        self.search_distance = 1.5   ### 내가 확인할 거리
        self.Same_distance = 0.25    ### 같은 물체인지 판별할 거리
        self.obs_exist_flag = False

        ### 동적 정적 판단 변수
        self.previoustime = None  # 기다릴 시간 기준
        self.nexttime = None      # 
        self.timefind = False
        self.lencount = 0
        self.ObsStatus = ""

        ## on/off 변수
        self.Labacorn_flag = False
        self.Rotary_flag = False
        self.Obs_flag = True       

        self.LidarData = LidarData()
        self.LidarControl = LidarControl() 

        rospy.on_shutdown(self.lidar_shutdown)
        rospy.spin()

    def callback(self, data):

        ## 변수 설정
        # obs_exist_flag = False
        count = 0
        left_roi_lidar = []
        right_roi_lidar = []
        for i in range(self.roi_degree_offset + 1):         ## 0 ~ degree까지
            left_roi_lidar.append([i, data.angle_min + i * data.angle_increment, -i, data.ranges[i]]) ## origin index, theta, angle, distance # 가운데 기준으로 왼쪽은 -angle로 지정
        j = self.roi_degree_offset
        for i in range(360 - self.roi_degree_offset, 360):  # 360-degree부터 360까지
            right_roi_lidar.append([i, data.angle_min + i * data.angle_increment, j, data.ranges[i]]) ## origin index, theta, angle, distance    # 가운데 기준으로 오른쪽은 +angle로 지정
            j -= 1

        roi_lidar = right_roi_lidar + left_roi_lidar  ## [origin index, theta, angle, distance] --> 차량 전면기준 오른쪽부터 저장
        
        for i in range(len(roi_lidar)):
            if roi_lidar[i][3] > self.search_distance : # 내가 확인할 거리보다 멀면 inf로 정의
                roi_lidar[i][3] = self.infinity
            # else:  ## 물체가 하나라도 limit distance 이내라면 물체가 있다고 판단
            #     count += 1
            # print(f"{roi_lidar[i][2]}도는 index : {roi_lidar[i][0]}, 거리 : {roi_lidar[i][3]} ")
        #print("다음")

        # speed = 1000
        # angle = 0.5
        # if count :
        #     obs_exist_flag = True

        # if obs_exist_flag :  # 물체가 하나라도 인식되면 참

        groupdata = self.grouping(roi_lidar)            ## grouping한 리스트 각각의 리스트 요소는 roi lidar index
        if len(groupdata) :
            # print(1)
            mgrpData = self.mergeObs(roi_lidar,groupdata)   ## 진짜로 같은 물첸지 다른 물첸지 판단까지함 각각의 리스트 요소는 roi lidar index
        else:
            mgrpData = []
        
        self.LidarData.GroupLen = len(mgrpData)
        # print(f"그룹된 갯수: {len(groupdata)}")
        # print("최종 그룹 갯수:",len(mgrpData))
        print("mgrpData : ",mgrpData)
        # print("다음")
        print(self.roi_degree_offset)

        #### 라바콘 
        if self.Labacorn_flag :
            self.LidarData.LabacornAngle = self.labacorn(roi_lidar,mgrpData)

        else:
            self.LidarData.LabacornAngle = -1

        #### 회전 로터리

        if self.Rotary_flag:
            if len(mgrpData):
                self.LidarData.RotaryAngle, self.LidarData.RotaryDis = self.rotary(roi_lidar,mgrpData)
            else:
                self.LidarData.RotaryAngle = -1
                self.LidarData.RotaryDis = -1
   
        else:
            self.LidarData.RotaryAngle = -1
            self.LidarData.RotaryDis = -1

        
        ### 동적 정적 판단 시작
        if self.Obs_flag:           
            
            if self.LidarData.ObStatus == 's' or  self.LidarData.ObStatus == 'm' :
                self.LidarData.ObStatus = 'N'            
                self.comparedata = None
                self.timefind = False
                print("판단됐다")
            elif self.LidarControl.Obstatus == "o":
                self.LidarData.ObStatus = 'N'            
                self.comparedata = None
                self.timefind = False
                
            if len(mgrpData):
                self.MvStOb(mgrpData)           
            else :
                self.LidarData.ObStatus = "N"  # N / s,m
            # elif self.LidarControl.Obstatus == "t":
            #     pass
            # else:
            #     print("판단 안됐다")
            # print("상태 : ",self.LidarData.ObStatus)
        else :
            print("장애물 판단 x")
            self.comparedata = None
            self.timefind = False
            self.LidarData.ObStatus = "N"

        print(f"obs : {self.LidarControl.obs_flag}, rotary : {self.LidarControl.Rotary_flag}, labacorn : {self.LidarControl.Labacorn_flag} ")
        # print(self.LidarData.ObStatus)
        
        ## publish
        # self.pub.publish(speed)
        # self.pub_angle.publish(angle)
        self.LidarPub.publish(self.LidarData)
    
    ## 그룹핑
    def grouping(self,data):  # grouping
        ## data = [origin index, theta, angle, distance]
        grouping_obs_flag = False
        oneObsIdx = []
        obs_pts = []

        for i in range(len(data)):

            if (data[i][3] == self.infinity):  ### 마지막 각도에서 infiny가 아니라면 물체가 있다고 인식이 안됨
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
        
            if (i == len(data)-1):  ### 마지막 물체가 infinity가 아니고 인식이 됐다면
                if grouping_obs_flag:
                        obs_pts.append(oneObsIdx)
                        grouping_obs_flag = False
                        continue
        
        return obs_pts  ## roi index만 있음 --> 그룹된 roi index
    
    ## 그룹핑 데이터 다시 한번 확인
    def mergeObs(self, data, gdata) :  # 유효범위 데이터와 그룹핑된 오브젝트를 집어넣음  #grouping한게 진짜 맞는지 한번 더 확인
        mgrpData = []        
        #data  == [origin idx, theta, tmp_degree, _data.ranges[origin idx]]
        # gdata == [[물체가 찾아 졌을 때 roi index] * 개별로 판별된 수]
        #print(len(gdata))
        for i in range(len(gdata)) :
            tmpgdata = gdata[i]
            equalCount = False
            for j in range(i+1, len(gdata)) :
                tmp_g0 = data[gdata[i][-1]]  ## 그루핑 물체를 찾아낸 마지막 roi index값을 집어넣는거
                tmp_g1 = data[gdata[j][0]]  ## 그루핑 물체를 찾아낸 첫번째 index값을 집어넣는거  --> 그루핑된 앞배열의 마지막 index의 거리와 뒷배열 첫번째 index의 거리가 limit same distance보다 작으면 같은 물체
                idx1_xy = calc_axis_xy(tmp_g0[1], tmp_g0[3], 0, self.search_distance) # 세타, 인식된 거리, 최소거리, 물체를 찾기 위한 최대거리  --> x,y좌표를 반환 받음
                idx2_xy = calc_axis_xy(tmp_g1[1], tmp_g1[3], 0, self.search_distance) # 세타, 인식된 거리, 최소거리, 물체를 찾기 위한 최대거리  --> x,y좌표를 반환 받음
                distance = calc_distance(idx1_xy, idx2_xy) ## 각각의 물체끼리의 거리를 구함

                if abs(distance) > self.Same_distance : ## 구한 거리가 같은 물체의 임계값을 넘어가면 다른 물체가 맞고 아니면 같은 물체에 i값 넣기 / 거리 0.20m
                    #print("여기 들어오니?",distance)
                    continue
                # print("first : ",tmpgdata)
                tmpgdata = tmpgdata + gdata[j]   ### 같은 물체로 판단되면 같은 list에 저장
                # print("second : ",tmpgdata) 
         
            for k in range(len(mgrpData)):  ## 이미 저장한 list인지 판별 
                tmp_pop0 = mgrpData[k].copy()
                tmp_pop1 = tmpgdata.copy()

                for tmp0 in tmp_pop0:
                    for tmp1 in tmp_pop1:
                        if tmp0 == tmp1 :
                            equalCount = True

            # print(tmpgdata)
            if not equalCount :
                mgrpData.append(tmpgdata)  ## roi index --> 진짜로 다른 물체인지 판단한 roi index

        return mgrpData

    ## 라바콘
    def labacorn(self,data,mgrpdata): # labacorn일때 주행각도 찾기
        # data : [origin idx, theta, tmp_degree, _data.ranges[origin idx]]*n개
        # mgrpdata : [[roi index] * 찾아진 물체 개수]
        tmp_g0 = data[mgrpdata[0][-1]]  # 군집화된 것중 오른쪽 끝의 물체 중 마지막 인덱스
        tmp_g1 = data[mgrpdata[-1][0]]  # 군집화된 것중 왼쪽 끝의 물체 중 첫번째 인덱스
        idx1_xy = calc_axis_xy(tmp_g0[1],tmp_g0[3],0,self.search_distance) # 오른쪽 처음 물체의 마지막 index의 x,y좌표
        idx2_xy = calc_axis_xy(tmp_g1[1],tmp_g1[3],0,self.search_distance) # 왼쪽 처음 물체의 첫 index의 x,y좌표
        
        position = idx1_xy[1] + idx2_xy[1]  ### 물체의 y로 각도 판별 --> 라이다가 뒤쪽을 기준으로 잡고 있어서 y가 좌우 x가 상하임
        angle = (position * 0.3849001794597506 ) + 0.5    #  (position+0.75)*2/3 <-- 최대값을 각도로만 계산 /// (position * 0.3849001794597506 ) + 0.5 <-- 최대값을 거리까지 계산

        print(f"각도는 : {angle}")
        print(f"왼쪽 idx : {mgrpdata[-1][0]} 오른쪽 idx : {mgrpdata[0][-1]}")
        return angle
    
    ## 회전 교차로
    def rotary(self,data,mgrpdata):
        # data : [origin idx, theta, tmp_degree, _data.ranges[origin idx]]*n개
        # mgrpdata : [[roi index] * 찾아진 물체 개수]
        #tmp_g0 = [0,0]  # 라이다 기준으로 (0,0)으로 잡아줌
        index = mgrpdata[0][-1] # 군집화된 물체의 마지막 index

        if index < 60: ## index가 60 미만이면 라이다 오른쪽의 물체가 잡힘
            tmp_g1 = data[mgrpdata[0][0]]  # 군집화된 것중 오른쪽 끝의 물체 중 첫번째 인덱스  

        else: ## 60 초과면 라이다의 왼쪽의 물체가 있음, 60은 가운데임
            tmp_g1 = data[mgrpdata[0][-1]]  # 군집화된 것중 왼쪽 끝의 물체 중 첫번째 인덱스
        
        idx1_xy = [0,0]   # tmp_g0    #idx1_xy = calc_axis_xy(tmp_g0[1],tmp_g0[3],0,self.search_distance) # 오른쪽 처음 물체의 마지막 index의 x,y좌표
        idx2_xy = calc_axis_xy(tmp_g1[1],tmp_g1[3],0,self.search_distance) # 왼쪽 처음 물체의 첫 index의 x,y좌표
        distance = calc_distance(idx1_xy,idx2_xy) ## 라이다와 물체간의 거리를 구함
        position = idx1_xy[1] + idx2_xy[1]  ### 물체의 y로 각도 판별 --> 라이다가 뒤쪽을 기준으로 잡고 있어서 y가 좌우 x가 상하임
        if index <= 60:
            angle = (position * 0.8247860988423228 ) + 0.5     # math.sin((300*np.pi/180) -np.pi) * 0.7
        else:
            # 물체가 왼쪽일때 (position+0.75)*2/3 <-- 최대값을 각도로만 계산 /// (position * 0.3849001794597506 ) + 0.5 <-- 최대값을 거리까지 계산
            angle = (position * 0.3849001794597506 ) + 0.5    

        # print(f"각도는 : {angle}, 거리 : {distance}")
        # print(f"왼쪽 idx : {mgrpdata[-1][0]} 오른쪽 idx : {mgrpdata[0][-1]}")
        return angle, distance

    ## 정적 동적판단
    def MvStOb(self,mgrpdata) : # move stop object
        # mgrpdata : [[roi index] * 찾아진 물체 개수]
        
        samecount = 0
        if not self.timefind : # timefind가 False면 실행
            self.previoustime = time.time()
            self.nexttime = time.time()            
            print("첫 실행이야")
            if len(mgrpdata[0]) >= 2:
                self.comparedata = mgrpdata[0] # mgrpdata의 첫 리스트의 마지막 index 복사
                self.timefind = True
            # print(self.timefind) 
        else:
            self.nexttime = time.time()
             
            if self.nexttime - self.previoustime >= 1.0: # 물체가 감지된 후 n초 지나고 정적,동적 판단                
                for i in self.comparedata:
                    for j in mgrpdata[0]:
                        if i == j :
                            samecount += 1
                print("compare : ",self.comparedata)
                print("count : ",samecount)
                if self.LidarData.ObStatus == "N":
                    print("판단하고싶어요")
                    if len(self.comparedata)-1 <= samecount <= len(self.comparedata) :
                        self.LidarData.ObStatus = "s"                
                    else:
                        self.LidarData.ObStatus = "m" 
                    print("obs status : ",self.LidarData.ObStatus)               
                
            else:
                if len(mgrpdata[0]) == 0:  ### 판단 도중에 장애물이 사라질 경우    
                    self.comparedata = None
                    self.timefind = False
                    self.LidarData.ObStatus = "n"
                else:
                    self.LidarData.ObStatus = "N"
        pass
        #return speed, angle

    def LidarCallback(self,data):
        self.LidarControl = data
        self.roi_degree_offset = data.roi_deg
        self.Obs_flag = data.obs_flag
        self.Rotary_flag = data.Rotary_flag
        self.Labacorn_flag = data.Labacorn_flag
        # print(self.LidarControl)
        pass

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
