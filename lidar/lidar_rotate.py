#! /usr/bin/env python3

# -*- coding:utf-8 -*-

import rospy
import numpy as np
import time
import math

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


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
        self.infinity = float("inf") ### inf는 뭐 정의가 되어있어 가능하다고 함
        self.search_distance = 1.5   ### 내가 확인할 거리
        self.Same_distance = 0.20    ### 같은 물체인지 판별할 거리
        self.obs_exist_flag = False  ### 물체가 인식되면 그룹핑부터 ~~ 
        self.rotate_flag = False

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


        ## 변수 설정
        obs_exist_flag = False
        count = 0
        left_roi_lidar = []
        right_roi_lidar = []
        for i in range(self.roi_degree_offset + 1):         ## 0 ~ degree까지
            left_roi_lidar.append([i, data.angle_min + i * angle_increment, -i, data.ranges[i]]) ## origin index, theta, angle, distance # 가운데 기준으로 왼쪽은 -angle로 지정
        j = self.roi_degree_offset
        for i in range(360 - self.roi_degree_offset, 360):  # 360-degree부터 360까지
            right_roi_lidar.append([i, data.angle_min + i * angle_increment, j, data.ranges[i]]) ## origin index, theta, angle, distance    # 가운데 기준으로 오른쪽은 +angle로 지정
            j -= 1

        roi_lidar = right_roi_lidar + left_roi_lidar  ## [origin index, theta, angle, distance] --> 차량 전면기준 오른쪽부터 저장
        
        for i in range(len(roi_lidar)):
            if roi_lidar[i][3] > self.search_distance : # 내가 확인할 거리보다 멀면 inf로 정의
                roi_lidar[i][3] = self.infinity
            else:  ## 물체가 하나라도 limit distance 이내라면 물체가 있다고 판단
                count += 1
            print(f"{roi_lidar[i][2]}도는 index : {roi_lidar[i][0]}, 거리 : {roi_lidar[i][3]} ")
        #print("다음")

        speed = 0
        angle = 0.5
        if count :
            obs_exist_flag = True

        if obs_exist_flag :  # 물체가 하나라도 인식되면 참

            groupdata = self.grouping(roi_lidar)            ## grouping한 리스트 각각의 리스트 요소는 roi lidar index
            mgrpData = self.mergeObs(roi_lidar,groupdata)   ## 진짜로 같은 물첸지 다른 물첸지 판단까지함 각각의 리스트 요소는 roi lidar index
            # print(f"그룹된 갯수: {len(groupdata)}")
            # print("최종 그룹 갯수:",len(mgrpData))
            print(mgrpData)
            # print("다음")
            angle, distance, index = self.rotate(roi_lidar,mgrpData)           
            if  0<= angle <= 0.5:
                #speed = 600 * angle + 700 # 1000~700                
                if distance >= 1.3: #1.3에 700~500이 젤 좋은거 같음 --> 거리의 범위를 바꾸면 각도 구하는 공식을 좀 손봐줘야함 + 속도도 다시 계산
                    if self.rotate_flag : ### 진입할때 왼쪽부터 인식이 되서 문제가 발생 --> 오른쪽이 인식이 된 후에 주행시 왼쪽을 인식하므로 오른쪽 인식되면 참
                        speed = 400 * angle + 300 #500 ~ 300
                    else:
                        speed = 0
                else:
                    speed = 0
            elif 0.5 < angle <=1:
                #speed = -600 * angle + 1300
                if distance >= 0.5:
                    speed = -400 * angle + 700
                    self.rotate_flag = True
                else:
                    speed = 0
            else:
                speed = 0    
            print(self.rotate_flag)
            
            print(speed)
            ## publish
        self.pub.publish(speed)
        self.pub_angle.publish(angle)
        
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
    
    def rotate(self,data,mgrpdata):
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
            # 물체가 오른쪽일때 각도 구하는 공식
            #angle = (position * 0.5773502691896258 ) + 0.5   # math.sin((300*np.pi/180) -np.pi) * 1            
            ''' 각도 구하는 공식 (x,y)좌표에서 y를 더해서 위치를 얻고 sin(라디안) * n으로 angle구하기
            (0,0.5)(0.8660254037844385,1) --> 0.8660254037844385 = math.sin((300*np.pi/180) -np.pi) * 1
            (1-0.5)/(0.8660254037844385 - 0) = (y-0.5)/(x -0) --> y = 0.5773502691896258 * x +0.5
            '''
            #angle = (position * 0.7216878364870323 ) + 0.5    # math.sin((300*np.pi/180) -np.pi) * 0.8
            angle = (position * 0.8247860988423228 ) + 0.5     # math.sin((300*np.pi/180) -np.pi) * 0.7
            #angle = (position * 0.9622504486493766 ) + 0.5
        else:
            # 물체가 왼쪽일때 (position+0.75)*2/3 <-- 최대값을 각도로만 계산 /// (position * 0.3849001794597506 ) + 0.5 <-- 최대값을 거리까지 계산
            angle = (position * 0.3849001794597506 ) + 0.5    

            ''' 각도 구하는 공식 (x,y)좌표에서 y를 더해서 위치를 얻고 sin(라디안) * n으로 angle구하기
            (-1.2990381056766578,0) (0,0.5) --> -1.2990381056766578 = math.sin((60*np.pi/180) -np.pi) * 1.5
            (0.5-0)/(0 - (-1.2990381056766578) = (y-0.5)/(x -0) --> y = 0.3849001794597506 * x +0.5
            '''
            #angle = (position * 0.5773502691896258) + 0.5

        print(f"각도는 : {angle}, 거리 : {distance}")
        print(f"왼쪽 idx : {mgrpdata[-1][0]} 오른쪽 idx : {mgrpdata[0][-1]}")
        return angle, distance, index
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