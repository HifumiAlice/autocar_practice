#!/usr/bin/env python
# -*- coding: utf-8 -*-

from json.encoder import INFINITY
import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import String

# Hz of /scan : 10 
# 앞 : 180, x - 앞(음수), 뒤(양수) y - 오른쪽(양수) 
class lidarReceiver( ) :
    def __init__(self) :
        rospy.loginfo("LiDAR Receiver")

        self.MOVING_STATUS = False                          # False - Obstable is not moving

        self.MEASURE_ANGLE = 40                             # 측정 각도

        self.FORWARD_ANGLE = 180 - self.MEASURE_ANGLE / 2   # Forward angle : -170 ~ 170
        self.SEARCH_ANGLE = 150                             # Angle for obstacle search

        self.STOP_RANGE = 0.7           # If obstable is moving, vehicle is stopped before 0.7m
        self.SEARCH_DISTANCE = 2.0      # Maximum distance to search obstacle 

        self.POINT_CNT = 5              # number of point of obstacle
        self.threshold_distance = 0.2   # 물체 사이의 최소 거리 (0.2m)

        # self.lscan = rospy.Subscriber("scan", LaserScan, self.lidarTestRecCallback)
        self.lscan = rospy.Subscriber("scan", LaserScan, self.obstacleDetectCallback)
        self.lidar_warning_pub = rospy.Publisher("lidar_detected_obstable", String, queue_size=5)


    def lidarTestRecCallback(self, _data) :
        # === 1. ranges에 해당하는 각도 구하기
        theta_rad = []
        for i in range(len(_data.ranges)) :
            tmp_theta = _data.angle_min + i * _data.angle_increment
            if ( tmp_theta >= _data.angle_max) :
                tmp_theta = _data.angle_max
            theta_rad.append(tmp_theta)

        # === 2. ranges 중 최소 거리 값 찾기
        min_idx = 0
        min_range = 1e9
        for i, distance in enumerate(_data.ranges) :
            if (distance < min_range) :
                min_range = distance
                min_idx = i

        rospy.loginfo("min range={} min_idx={}".format(min_range, min_idx))

        # === 3. 해당 거리값과 각도값을 이용해서 x, y로 변환하기
        x = min_range * math.cos(theta_rad[min_idx])
        y = min_range * math.sin(theta_rad[min_idx])

        # === 4. r, theta, x, y 출력
        min_deg = theta_rad[min_idx] * 180 / math.pi
        rospy.loginfo("Mininum range(r, theta, x, y)=({:.3f}, {:.3f}, {:.3f}, {:.3f}".format(min_range, min_deg, x, y))

    # 차량 정면엥 있는 물체 확인 --> 경고 메시지 출력
    #  1. 정면에 해당하는 부분 정하기 -170 ~ 170
    #  2. 거리 기준으로 y 


    def obstacleDetectCallback(self, _data) :
        cluster = []
        objectsLoc = []
        emptySpace = []

        # angle, theta_deg, theta_idx = self.genAngle(_data, self.FORWARD_ANGLE)
        angle, theta_idx = self.genAngle(_data, self.FORWARD_ANGLE)

        # rospy.loginfo("angle = {}, theta_deg({}) = {} , theta_idx={}".format(angle, len(theta_deg), theta_deg, theta_idx))

        # 현의 길이 (70cm, 2m, 3)
        chord_length_stop = self.obstacle_sizeIncrement(self.STOP_RANGE)
        chord_lane_change = self.obstacle_sizeIncrement(self.SEARCH_DISTANCE)

        rad_forward_angle = self.MEASURE_ANGLE * math.pi / 180 

        # SEARCH_DISTANCE 에서의 물게 갯 수 확인 - 갯수, 움직이고 있는지
        #  - 20도, 30도 일 때 현의 길이 계산
        #      - 20도 일 때 --> 2m 거리의 현의 길이(67cm), 3m 거리(1m)
        #      - 30도 일 때 --> 2m 거리의 현의 길이(1m), 3m 거리(1.55)
        # rospy.loginfo("Chord length of search distance({}m,{}deg) = {}".format(self.SEARCH_DISTANCE, self.FORWARD_ANGLE, 2 * self.SEARCH_DISTANCE * math.sin(rad_forward_angle/2)))

        self.detect_max_distance = 3.0

        firstObjIdx = 0
        length_data = len(theta_idx)
        points = 1
        empty = 0
        n = 1
        d = INFINITY
        idx = 0
        i = 0

        # 가장 왼쪽에 있는 물체 확인
        for i in range(length_data) :
            idx = theta_idx[i]
            distance = _data.ranges[idx]
            if (distance < self.detect_max_distance) :
                break
        
        theta0 = theta = _data.angle_min + theta_idx[i] * _data.angle_increment
        x0 = x = _data.ranges[idx] * math.cos(theta)
        y0 = y = _data.ranges[idx] * math.sin(theta)
        idx0 = idx
        no_cluster = 1
        rospy.loginfo("range of first obs({})({}) = {}({})".format(i, idx, distance, theta0 * 180 / math.pi))
        rospy.loginfo("i={}, theta={}, x0={}, y0={}, idx0={}, idx={}".format(i, theta, x0, y0, idx0, idx))

        for j in range(i, length_data) :
            idx1 = theta_idx[j]
            theta1 = _data.angle_min + idx1 * _data.angle_increment
            x1 = y1 = 0
            
            if _data.ranges[idx1] < self.detect_max_distance :
                x1 = _data.ranges[idx1] * math.cos(theta1)
                y1 = _data.ranges[idx1] * math.sin(theta1)
                d = math.sqrt(pow((x1-x), 2) + pow((y1-y), 2))  # 이전 point와 거리 계산

            rospy.loginfo("j={}, i={}, idx1={}, range={}, x={}, y={},x1={}, y1={}, d={}".format(j,i,idx1, _data.ranges[idx1], x, y, x1, y1, d))

            if ( d < self.threshold_distance ) :         # threshold_distance보다 가까운 거리는 같은 물체로 인식
                points += 1
            else :
                if d > 0.4 :
                    empty += 1

                rospy.loginfo("idx0({}), idx1({})".format(idx0, idx1))

                objectsLoc.append([[idx0, idx1], [x0, y0], [x1, y1], [theta0 * 180 / math.pi, theta1 * 180 / math.pi], [_data.ranges[idx0], _data.ranges[idx1]]])    # Object의 좌표 추가
                cluster.append(points)
                points = 1
                n += 1
                idx0 = idx1
                theta0 = theta1
                x0 = x1
                y0 = y1

            theta = theta1
            idx = idx1
            x = x1
            y = y1

        
        emptySpace.append([[x,y], [x1,y1]])

        rospy.loginfo("Total no. of clusters : {}".format(len(cluster)))
        for j in range(len(cluster)) :
            rospy.loginfo("No. of points in cluster {} is : {}".format(j+1, cluster[j]))
        rospy.loginfo("Total no. of empty space through which car can pass: {}".format(empty))
        rospy.loginfo("Objects Location : {}".format(objectsLoc))
        rospy.loginfo("Empty Location : {}".format(emptySpace))
            

        #  2. 


        # rospy.loginfo("obs size : ({})={}, ({}) = {}".format(self.STOP_RANGE, obs_size_inc_stop, self.SEARCH_DISTANCE, obs_size_inc_lane_change))

        # min_distance = 100.0
        # point_cnt = 0
        # for i, distance in enumerate(_data.ranges) :
        #     if ( self.FORWARD_ANGLE <= abs(theta_deg[i]) <= 180 ) :
        #         if distance < self.STOP_RANGE :
        #             point_cnt += 1
        #         if min_distance > distance :
        #             min_distance = distance

        # if (point_cnt >= self.POINT_CNT) :
        #     self.lidar_warning_pub.publish("warning")
        #     rospy.loginfo("Warning - Distance({})".format(min_distance))
        # else :
        #     self.lidar_warning_pub.publish("safe")
        #     rospy.loginfo("safe - Distance({})".format(min_distance))

    def genAngle(self, _data, angle ) :
        self.angle_increment = _data.angle_increment

        std_theta = angle * math.pi / 180
        theta_deg = []
        theta_idx = []
        rospy.loginfo("length of _data = {}, angle_increment={}".format(len(_data.ranges), _data.angle_increment*180/math.pi))

        no_data = len(_data.ranges)

        for i in range(no_data) :
            idx = int((i + no_data / 2) % no_data)
            tmp_theta = _data.angle_min + (idx * _data.angle_increment)
            if (abs(tmp_theta) >= std_theta) :
                theta_idx.append(idx)

        return angle, theta_idx

    # def genAngle(self, _data, angle ) :
    #     self.angle_increment = _data.angle_increment

    #     std_theta = angle * math.pi / 180
    #     theta_deg = []
    #     theta_idx = []
    #     rospy.loginfo("length of _data = {}, angle_increment={}".format(len(_data.ranges), _data.angle_increment*180/math.pi))

    #     no_data = len(_data.ranges)

    #     for i in range(no_data) :
    #         idx = int((i + no_data / 2) % no_data)
    #         tmp_theta = _data.angle_min + (idx * _data.angle_increment)
    #         if (abs(tmp_theta) >= std_theta) :
    #             theta_deg.append(tmp_theta * 180 / math.pi)
    #             theta_idx.append(idx)

    #     return angle, theta_deg, theta_idx

    def obstacle_sizeIncrement(self, radius) :
        return 2 * radius * math.sin(self.angle_increment / 2)

    def countObstacle(self, _data) :
        pass

    # def obstacleDetectCallback(self, _data) :
    #     theta_deg = self.genAngle(_data)

    #     rospy.loginfo("theta_deg = {}".format(theta_deg))

    #     min_distance = 100.0
    #     point_cnt = 0
    #     for i, distance in enumerate(_data.ranges) :
    #         if ( self.FORWARD_ANGLE <= abs(theta_deg[i]) <= 180 ) :
    #             if distance < self.STOP_RANGE :
    #                 point_cnt += 1
    #             if min_distance > distance :
    #                 min_distance = distance

    #     if (point_cnt >= self.POINT_CNT) :
    #         self.lidar_warning_pub.publish("warning")
    #         rospy.loginfo("Warning - Distance({})".format(min_distance))
    #     else :
    #         self.lidar_warning_pub.publish("safe")
    #         rospy.loginfo("safe - Distance({})".format(min_distance))

    def clustering(self, _data) :
        theta_deg = self.genAngle(_data)
        idx = 0

        for i, distance in enumerate(_data.ranges) :
            if ( self.SEARCH_ANGLE <= abs(theta_deg[i]) <= 180 ) :
                if distance < self.SEARCH_DISTANCE :
                    idx = i
                    break

        # initialisinig the first point i.e. x0 and y0 
        x0 = x =_data.ranges[i]*math.cos(theta_deg[idx])
        y0 = y =_data.ranges[i]*math.sin(theta_deg[idx])

        i+=1
        points=1
        n=-1
        empty=0
        






    def status_print(self) :
        rospy.loginfo("LiDAR-")



# (r, theta) --> (x, y)로 변환
def calc_axis_xy(_theta, _distance, _min_range, _max_range) :
    if _min_range <= _distance <= _max_range :
        x = np.cos(_theta) * _distance
        y = np.sin(_theta) * _distance
        return (x, y)
    else :
        return (0, 0)


def run():
    rospy.init_node("Lidar_Receiver")
    lidarReceiver()
    rospy.spin()

if __name__ == '__main__' :
    run()