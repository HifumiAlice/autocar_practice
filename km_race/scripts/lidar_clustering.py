#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cmath import inf
from json.encoder import INFINITY
from tkinter.messagebox import NO
import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from km_race.msg import filteredlidar, coords, lidar_measure, obs_theta, obs_range

# Hz of /scan : 10 
# 앞 : 180, x - 앞(음수), 뒤(양수) y - 오른쪽(양수) 
# 2m, 20도 --> 측정하는 현의 길이 0.7m --> 물체 확인에 문제 없음
# 2m, 40도 --> 측정하는 현의 길이 1.37m --> 
# 반지름	각도	Radian	        현길이
#  2.0	   40	0.698131701	  1.368080573
#  2.0	   20	0.34906585	  0.694592711
#  0.7	   20	0.34906585	  0.243107449
#  0.7	  120	2.094395102	  1.212435565
#  0.7	   30	0.523598776	  0.362346663
#  0.7	   40	0.698131701	  0.478828201
#  1.5	   40	0.698131701	  1.02606043

# 라바콘 주행시에는 가까운 곳에 있는 라바콘을 확인해야 하므로 1m 이내, 60도로 변경 <== control.py 에서 측정하기 위한 데이터 전달이 필요함.

# lidar 의 측정범위 설정을 위한 publisher 
# mode 반지름	각도	Radian	        현길이
#  0    3.0	   20	0.34906585	  1.041889066 --> 물체 판별, 움직임에 적합
#  1    2.0	   40	0.698131701	  1.368080573 --> 물체 판별, 움직임에 적합
#  2    2.0	   20	0.34906585	  0.694592711 --> 직진시 적합
#  3    0.7	   60	1.047197551	  0.7         --> 움직이는 물체 있는 경우 물체가 없어질 때까지 stop
#  4    0.7	  120	2.094395102	  1.212435565 --> 라바콘 주행중에 적합 (라바콘 지난 경우 체크)

class lidarClustering( ) :
    def __init__(self) :
        rospy.loginfo("[lidar_clustering] publish(/lidar_clustering), subscribe(/lidar_measure)")

        self.MOVING_STATUS = False                          # False - Obstable is not moving
        # self.DEFAULT_ANGLE = 40
        # self.DEFAULT_DISTANCE = 2
        self.DEFAULT_ANGLE = 120                            # for test ==================
        self.DEFAULT_DISTANCE = 1.5                         # for test ==================

        self.MEASURE_ANGLE = self.DEFAULT_ANGLE             # 측정 각도 폭

        self.FORWARD_ANGLE = 170                            # 직진하는데 사용하는 각도
        self.SEARCH_ANGLE = 180 - self.MEASURE_ANGLE / 2    # 물체를 찾는 최대 각도 MEASURE_ANGLE=40 --> 160도

        self.STOP_RANGE = 0.7           # If obstable is moving, vehicle is stopped before 0.7m
        self.SEARCH_DISTANCE = self.DEFAULT_DISTANCE        # 물체를 찾기위한 최대 거리 

        self.SAME_OBS_DEV_LIMIT = 0.10                     # angle_increment당 물체까지의 거리가 10cm이상 차이나는 경우, 다른 물체임
        self.obs_exist_flag = False

        # self.lscan = rospy.Subscriber("scan", LaserScan, self.lidarTestRecCallback)
        self.lscan = rospy.Subscriber("scan", LaserScan, self.clusteringCallback)
        self.measure_mode = rospy.Subscriber("lidar_measure", lidar_measure, self.MeasureModeCallback)

        self.lidar_clustering_pub = rospy.Publisher("lidar_clustering", filteredlidar, queue_size=5)

    # publish data : 
    #     obstacle 수
    #     obstacle의 크기
    #     obstable좌표 [[[xr, yr],[xl, yl]],....]
    #     가장 가까운 obs의 번호 (오른쪽이 0번)
    #     다음으로 가까운 obs의 번호

    def clusteringCallback(self, _data) :

        # rospy.loginfo("Row LiDAR Data:{}".format(_data))

        # 1. Filtering
        filteredData = self.filtering(_data, self.SEARCH_ANGLE, self.SEARCH_DISTANCE)

        # rospy.loginfo("[Clustering] fliteredData={}".format(filteredData))
        # rospy.loginfo("[LiDAR Cluster][clustering CB] obs_exist_flag({})".format(self.obs_exist_flag))
        # rospy.loginfo("[lidar_clustering][clusteringCallback] MEASURE_ANGLE({}), SEARCH_DISTANCE({})".
        #     format(self.MEASURE_ANGLE, self.SEARCH_DISTANCE))

        if self.obs_exist_flag :
            # 2. Grouping
            groupedData, groupedSpace = self.grouping(filteredData)
            # rospy.loginfo("[clusteringCB] angle({}), groupedData={}".format(self.MEASURE_ANGLE, groupedData))

            mgrpData = self.mergeObs(filteredData, groupedData)
            # rospy.loginfo("[clusteringCB] distance({}), mgrpData={}".format(self.SEARCH_DISTANCE, mgrpData))

            # 3. Analysis & publish
            self.analysis_pub(_data, filteredData, mgrpData, groupedSpace)
        else :
            self.empty_data_pub()
        
    def MeasureModeCallback(self, _data) :
        # Measure를 위한 angle와 distance가 pub가 안 된 경우, Default 로 설정 
        if _data is None :
            self.MEASURE_ANGLE = self.DEFAULT_ANGLE
            self.SEARCH_DISTANCE = self.DEFAULT_DISTANCE
            self.SEARCH_ANGLE = 180 - self.MEASURE_ANGLE / 2
        else :
            self.MEASURE_ANGLE = _data.measure_degree
            self.SEARCH_DISTANCE = _data.measure_length
            self.SEARCH_ANGLE = 180 - self.MEASURE_ANGLE / 2

        # rospy.loginfo("[lidar_clustering][MeasureModeCallback] MEASURE_ANGLE({}), SEARCH_DISTANCE({})".
        #     format(self.MEASURE_ANGLE, self.SEARCH_DISTANCE))

    # validRangeData --> filteredData
    #   - idx, theta, degree, distance
    #   [1217, 2.813732022885233, 161.2149695921314, inf], 
    #   [1218, 2.8186254696920514, 161.49534344143387, inf], 
    #   [1219, 2.8235189164988697, 161.77571729073634, 1.1150000095367432], 
    #   [1220, 2.828412363305688, 162.0560911400388, 1.1150000095367432], 
    #   [1221, 2.833305810112506, 162.33646498934124, 1.1150000095367432], 

    def filtering(self, _data, valid_angle, max_distance) :
        # Crop : remove all points that are outside boundary
        limit_theta = valid_angle * math.pi / 180
        count = 0
        self.obs_exist_flag = False

        validRangeData = []
        no_data = len(_data.ranges)

        for i in range(no_data) :
            idx = int((i + no_data / 2) % no_data)
            tmp_theta = _data.angle_min + (idx * _data.angle_increment)
            if (abs(tmp_theta) >= limit_theta) :
                tmp_degree = tmp_theta * 180 / math.pi

                if (_data.ranges[idx] > max_distance) :
                    if idx == 1284 and _data.ranges[idx-1] <= max_distance:
                        validRangeData.append([idx, tmp_theta, tmp_degree, _data.ranges[idx-1]])
                    else :
                        validRangeData.append([idx, tmp_theta, tmp_degree, inf])
                else :
                    count += 1
                    validRangeData.append([idx, tmp_theta, tmp_degree, _data.ranges[idx]])

        if count > 5 :
            self.obs_exist_flag = True

        return validRangeData

    # obs_pts ->  groupedData -> obsData, space_pts -> groupedSpace -> spaceData 
    # [6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,.... , 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36], 
    # [74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91], 
    # [93, 94, 95, 96, 97, 98, 99, 100, 101, ... , 115, 116, 117, 118, 119, 120, 121, 122]]

    def grouping(self, ftdData) :
        obs_pts = []
        oneObsIdx = []
        grouping_obs_flag = False

        space_pts = []
        oneSpaceIdx = []
        grouping_space_flag = False

        obs_dist = inf
        for i in range(len(ftdData)) :
            orgidx, theta, degree, dist = ftdData[i]
            # rospy.loginfo("[Clustering-segmentation] i({}), orgidx({}), theta({}), degree({}), dist({})".format(i, orgidx, theta, degree, dist))
            if dist == inf :
                if grouping_obs_flag :          # grouping을 하고 있는 경우 --> grouping된 데이터를 obs_pts에 저장
                    obs_pts.append(oneObsIdx)
                    grouping_obs_flag = False
                continue

            if grouping_obs_flag == False :     # grouping의 첫번째 데이터
                oneObsIdx = []
                oneObsIdx.append(i)
                obs_dist = dist
                grouping_obs_flag = True
            else :                              # grouping하고 있는 두번째 데이터 부터
                # 다른 물체로 판명되는 경우
                if abs(obs_dist - dist) > self.SAME_OBS_DEV_LIMIT :
                    obs_pts.append(oneObsIdx)
                    oneObsIdx = []
                    oneObsIdx.append(i)
                # 같은 물체인 경우 --> 현재의 거리를 기준으로 삼고 --> group에 추가
                else :
                    oneObsIdx.append(i)

                obs_dist = dist

        no = len(ftdData)
        for i in range(no) :
            orgidx, theta, degree, dist = ftdData[i]
            # rospy.loginfo("[Clustering-segmentation] i({}), orgidx({}), theta({}), degree({}), dist({})".format(i, orgidx, theta, degree, dist))
            if dist == inf :                            # space 인 경우
                if grouping_space_flag :                # grouping 하고 있었을 경우 --> 계속 oneSpaceIdx 에 추가
                    oneSpaceIdx.append(i)
                    if (i == no - 1) :
                        space_pts.append(oneSpaceIdx)
                else :                                  # grouping 하고 있지 않았을 경우 --> oneSpaceIdx를 초기화 하고, 항목 추가
                    oneSpaceIdx = []
                    oneSpaceIdx.append(i)
                    grouping_space_flag = True          # space grouping 중으로 설정

            else :                                      # Obstable이 있는 경우                                     
                if grouping_space_flag :                # space를 grouping을 하고 있는 경우 --> grouping된 데이터를 space_pts에 저장
                    space_pts.append(oneSpaceIdx)
                    grouping_space_flag = False
                continue

        return obs_pts, space_pts


    def mergeObs(self, filteredData, gdata) :
        mgrpData = []

        for i in range(len(gdata)) :
            tmpgdata = gdata[i]
            for j in range(i+1, len(gdata)) :
                tmp_g0 = filteredData[gdata[i][0]]
                tmp_g1 = filteredData[gdata[j][0]]

                idx1_xy = calc_axis_xy(tmp_g0[1], tmp_g0[3], 0, self.SEARCH_DISTANCE)
                idx2_xy = calc_axis_xy(tmp_g1[1], tmp_g1[3], 0, self.SEARCH_DISTANCE)

                distance = calc_distance(idx1_xy, idx2_xy)

                if distance > self.SAME_OBS_DEV_LIMIT :
                    continue

                tmpgdata = tmpgdata + gdata[j]
        
            mgrpData.append(tmpgdata)

        return mgrpData
            

    # calculate Obstable size
    #   obsData = [첫번째OBS의인덱스리스트, 두번째OBS의인덱스리스트, ....]
    #   첫번째OBS의인덱스리스트 = [인덱스1, 인덱스2, ...., idxN]
    #   인덱스x = _data의 인덱스
    # 

    # publish data : 
    #     obstacle 수
    #     obstacle의 크기
    #     obstacle좌표 [[[xr, yr,xl, yl]],....]
    #     obstacle Theta [[right_theta, left_theta]]
    #     가장 가까운 obs의 번호 (오른쪽이 0번)
    #     다음으로 가까운 obs의 번호

    def analysis_pub(self, _data, filteredData, obsData, spaceData) :
        pub_data = filteredlidar()

        # obs의 크기 계산
        no_obs = len(obsData)

        obs_size_list = []
        obs_coord_list = []

        for ns in range(no_obs) :
            w = len(obsData[ns])          # length of each obs
            # obs의 오른쪽 시작점
            idxR = obsData[ns][0]         # obs의 오른쪽 점의 idx
        
            dataR = filteredData[idxR]    # 첫번째 obs point의 filteredData : _dataIDX,  theta, degree, _data.ranges
            
            # filteredData=[0, -3.1415927410125732, -180.00000500895632, 1.1970000267028809]
            # filteredData=[41, -2.940961421933025, -168.50467718755567, 2.243000030517578]
            # rospy.loginfo("filteredData-dataR={}".format(dataR))

            rightPoint = calc_axis_xy(dataR[1], dataR[3], 0, self.SEARCH_DISTANCE)

            # obs의 왼쪽 끝점
            idxL = obsData[ns][w-1]       # obs의 왼쪽 점의 idx
            dataL = filteredData[idxL]

            # rospy.loginfo("filteredData-dataL={}".format(dataL))

            leftPoint = calc_axis_xy(dataL[1], dataL[3], 0, self.SEARCH_DISTANCE)

            obs_size = calc_distance(rightPoint, leftPoint)

            points = rightPoint
            points.extend(leftPoint)

            obs_size_list.append(obs_size)
            obs_coord_list.append(points)

            pub_data.obs_coord.append(coords(rightX=rightPoint[0], rightY=rightPoint[1], leftX=leftPoint[0],leftY=leftPoint[1]))
            pub_data.theta.append(obs_theta(right=dataR[1], left=dataL[1]))
            pub_data.ranges.append(obs_range(right=dataR[3], left=dataL[3]))
        
        pub_data.sizes_of_obstacle = obs_size_list

        # space 찾기
        #   obs중 가장 앞에 있는 것 2개를 찾는다.
        #      - 가장 가까운 것을 찾는다.
        #      - 두번째 가까운 것을 찾는다.
        #      - 비교시 가운데 것으로 비교한다.
        #
        #   2개 사이의 거리를 측정한다.
        #      - 오른쪽의 마지막 point와 왼쪽의 첫번째 포인트로 거리를 측정한다.
        #      - 공간의 폭이 0.25이상이어야 한다. 이하일 경우 stop한다. (controller.py)
        #      - 2좌표의 중앙으로 이동하면 된다. (controller.py)

        centerIdx = []          # 물체의 가운데 포인트에 대한 filteredData의 index
        for i in range(len(obsData)) : 
            w = len(obsData[i])
            centerIdx.append(obsData[i][int(w/2)])

        # ====== 여기서부터 수정 =================================================================

        # if len(centerIdx) >= 2 :
        #     smallest_group = 0

        #     # 가장 가까운 물체를 확인하여 인덱스를 smallest에 저장 ========================

        #     smallest = centerIdx[0]
        #     # rospy.loginfo("smallest r={}".format(filteredData[smallest][3]))
        #     for i in range(1, len(centerIdx)) :
        #         r0 = filteredData[smallest][3]
        #         r1 = filteredData[centerIdx[i]][3]
        #         # rospy.loginfo("1-centerIdx[{}]={} r={}".format(i, centerIdx[i], r1))

        #         if r0 > r1 :
        #             smallest = centerIdx[i]
        #             smallest_group = i

        #     # rospy.loginfo("result smallest({}) group={} r={}".format(smallest, smallest_group, filteredData[smallest][3]))

        #     # 두번째로 가까운 물체를 확인하여 인덱스를 next에 저장
        #     # next 의 초기값 설정
        #     if smallest == centerIdx[0] :
        #         next = centerIdx[1]
        #         next_group = 1
        #     else :
        #         next = centerIdx[0]
        #         next_group = 0

        #     # rospy.loginfo("next = {}".format(next))
        #     if len(centerIdx) > 2 :
        #         for i in range(len(centerIdx)) :
        #             if smallest == centerIdx[i] :
        #                 continue

        #             r0 = filteredData[next][3]
        #             r1 = filteredData[centerIdx[i]][3]
        #             # rospy.loginfo("2-centerIdx[{}]={} r={}".format(i, centerIdx[i], r1))

        #             if r0 > r1 :
        #                 next = centerIdx[i]       
        #                 next_group = i   

        #     # rospy.loginfo("smallest({})={} group={} next({})={}".format(smallest, filteredData[smallest][3], next_group, next, filteredData[next][3]))
        # else :
        #     smallest_group = 0
        #     next_group = 0

        # rospy.loginfo("Obs_size({}), centerIdx({})".format(obs_size_list, centerIdx))

        # 가장 가까운 거리의 2개 obs group 번호를 publish한다.
        # pub_data.smallest_obs = smallest_group
        # pub_data.next_obs = next_group
        # pub_data.no_obstacle = no_obs

        # =============================== 여기까지 수정 ==========================


        # centerIDX
        # centerIDX=[21, 83, 108]
        # rospy.loginfo("centerIDX={}".format(centerIdx))
        # filteredData : _dataIDX,  theta, degree, _data.ranges

        # smallest_group : 오른쪽 물체
        # next_group : 왼쪽 물체

        if len(centerIdx) >= 2 :

            rightObs = centerIdx[0]         # 가장 왼쪽에 있는 물체의 인덱스를 0으로 초기화
            leftObs = centerIdx[-1]            # 가장 오른쪽에 있는 물체의 인덱스를 물체의 갯수(마지막 물체)로 초기화

            rightGroup = 0
            leftGroup = len(centerIdx) - 1

            # rospy.loginfo("filteredData={}".format(filteredData))
            # rospy.loginfo("filteredData[{}]={}".format(rightObs, filteredData[rightObs]))
            # rospy.loginfo("=== centerIDX({}), right({}), left({})".format(centerIdx, rightObs, leftObs))

            for i in range(1, len(centerIdx)-1) :
                r0 = filteredData[rightObs][3]
                r1 = filteredData[centerIdx[i]][3]

                if i == leftGroup :
                    break

                if filteredData[centerIdx[i]][1] < 0 :
                    break

                if filteredData[centerIdx[i]][3] > filteredData[centerIdx[i+1]][3] :
                    break

                if r0 > r1 :
                    rightObs = centerIdx[i]
                    rightGroup = i

            # rospy.loginfo("right obs r({}), theta({}), degree({})".format(filteredData[rightObs][3], filteredData[rightObs][1], filteredData[rightObs][2]))
            # rospy.loginfo("rightObs filteredData[{}]={}".format(rightObs, filteredData[rightObs]))

            # 가장 왼쪽 물체 확인

            for i in range(len(centerIdx)-2, rightGroup, -1) :
                r0 = filteredData[leftObs][3]
                r1 = filteredData[centerIdx[i]][3]

                if rightGroup == i :
                    break

                if filteredData[centerIdx[i]][1] > 0 :
                    break

                if filteredData[centerIdx[i]][3] > filteredData[centerIdx[i-1]][3] :
                    break

                if r0 > r1 :
                    leftObs = centerIdx[i]        
                    leftGroup = i

            # rospy.loginfo("leftObs filteredData[{}]={}".format(leftObs, filteredData[leftObs]))
            rospy.loginfo("no({}), leftobs[{}]={}, rightobs[{}]={}".format(no_obs, leftObs, filteredData[leftObs], rightObs, filteredData[rightObs]))

        else :
            rightGroup = 0
            leftGroup = 0

        pub_data.smallest_obs = rightGroup
        pub_data.next_obs = leftGroup
        pub_data.no_obstacle = no_obs

        # rospy.loginfo("pub_data.ranges({})".format(pub_data.ranges))


        self.lidar_clustering_pub.publish(pub_data)

    def empty_data_pub(self) :
        pub_data = filteredlidar()
        pub_data.no_obstacle = 0
        pub_data.smallest_obs = 0
        pub_data.next_obs = 0

        self.lidar_clustering_pub.publish(pub_data)

# (r, theta) --> (x, y)로 변환
def calc_axis_xy(_theta, _distance, _min_range, _max_range) :
    if _min_range <= _distance <= _max_range :
        x = np.cos(_theta) * _distance
        y = np.sin(_theta) * _distance
        return [x, y]
    else :
        return [0, 0]

def calc_distance(pr, pl) :
    return math.sqrt(pow((pr[0] - pl[0]), 2) + pow((pr[1] - pl[1]), 2))

def run():
    rospy.init_node("Lidar_Clustering")
    lidarClustering()
    rospy.spin()

if __name__ == '__main__' :
    run()