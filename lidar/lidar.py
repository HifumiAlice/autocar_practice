#! /usr/bin/env python3

#-*- coding:utf-8 -*-

import rospy
import numpy as np

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

        self.roi_degree_offset = 10  ### 라이다 볼 인덱스 값


        rospy.on_shutdown(self.lidar_shutdown)
        rospy.spin()

    def callback(self, msg):

        ########## lidar ##########
        angle_min_degree = msg.angle_min / np.pi * 180
        angle_max_degree = msg.angle_max / np.pi * 180    
        '''
        원하는 각도를 입력해서 라이다의 각도랑 맞출려면 
        angle_radian = angle_degree *(np.pi/180)
        원하는 라디안을 입력해서 각도를 나타내려면 
        angle_degree = angle_radian * (180/np.pi)
        '''
        #print("각도 증가",msg.ranges[0])
        #print("angle_min_degree : {}, angle_max_degree : {} ".format(angle_min_degree,angle_max_degree))

        #angle_increment_degree = msg.angle_increment / np.pi * 180
        #print("angle_increment : ", angle_increment_degree)

        # lidar_len = len(msg.ranges)
        # print("갯수 : ",lidar_len)
        roi_ranges_left = list(msg.ranges[0:self.roi_degree_offset + 1])       
        roi_ranges_right = list(msg.ranges[360-self.roi_degree_offset : 360])
        roi_ranges_all = roi_ranges_right + roi_ranges_left
        roi_ranges_right = roi_ranges_right[::-1]

        for i in range(0,20):
            if i <= 9:
                print(f"오른쪽 {i}번째 값 : {roi_ranges_all[i]} ")
            else:
                print(f"왼쪽 {i-9}번째 값 : {roi_ranges_all[i]}")
        # for i in range(0,self.roi_degree_offset):
        # #    print("왼쪽  {}번째 인덱스 lidar값: {}".format(i,roi_ranges_left[i]))
        #    print("오른쪽 {}번째 인덱스 lidar값: {}".format(i,roi_ranges_right[i]))
        #    #count += 1

        print("다음")

        ########## lidar를 통해 간단하게 장애물 만나면 정지 없으면 직진하는 코드 짜기 ##########
        # count = 0
        # for i in range(0,self.roi_degree_offset):
        #     if roi_ranges_left[i] < 1.5:
        #         count += 1
        #         print("왼쪽  {}번째 인덱스 lidar값: {}".format(i,roi_ranges_left[i]))
        #         #print(1)
        
        #     if roi_ranges_right[i] < 1.5:
        #         print("오른쪽 {}번째 인덱스 lidar값: {}".format(i,roi_ranges_right[i]))
        #         count += 1

        # if count > 7:
        #     speed_pub = 0.0
        #     print("위험!!")

        # elif count <= 7:
        #     speed_pub = 1000
            

        # self.pub.publish(speed_pub)

        #rate.sleep()

    
    def lidar_shutdown(self):
        print("Lidar is Dead !!")


if __name__ == "__main__":

    ld = lidar()