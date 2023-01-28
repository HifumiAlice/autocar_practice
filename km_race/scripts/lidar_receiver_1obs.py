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
        theta_deg = self.genAngle(_data)
        # rospy.loginfo("theta_deg = {}".format(theta_deg))

        min_distance = 100.0
        point_cnt = 0
        for i, distance in enumerate(_data.ranges) :
            if ( self.FORWARD_ANGLE <= abs(theta_deg[i]) <= 180 ) :
                if distance < self.STOP_RANGE :
                    point_cnt += 1
                if min_distance > distance :
                    min_distance = distance

        if (point_cnt >= self.POINT_CNT) :
            self.lidar_warning_pub.publish("warning")
            rospy.loginfo("Warning - Distance({})".format(min_distance))
        else :
            self.lidar_warning_pub.publish("safe")
            rospy.loginfo("safe - Distance({})".format(min_distance))

    def genAngle(self, _data) :
        theta_deg = []
        for i in range(len(_data.ranges)) :
            tmp_theta = _data.angle_min + i * _data.angle_increment
            if ( tmp_theta >= _data.angle_max) :
                tmp_theta = _data.angle_max
            theta_deg.append(tmp_theta * 180 / math.pi)
        return theta_deg

def status_print(self) :
        rospy.loginfo("LiDAR-")

# (r, theta) --> (x, y)로 변환
def calc_axis_xy(_theta, _distance, _min_range, _max_range) :
    if _min_range <= _distance <= _max_range :
        x = np.cos(_theta) * _distance
        y = np.sin(_theta) * _distance
        return [x, y]
    else :
        return [0, 0]


def run():
    rospy.init_node("Lidar_Receiver")
    lidarReceiver()
    rospy.spin()

if __name__ == '__main__' :
    run()