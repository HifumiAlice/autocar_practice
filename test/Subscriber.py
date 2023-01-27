#! /usr/bin/env python3

# -*- coding:utf-8 -*-

# import rospy
# from std_msgs.msg import Int32
# from geometry_msgs.msg import Twist
# from turtlesim.msg import Pose
# import numpy as np
# rospy.init_node('sub_test')
# cmd_vel = Twist()

# degree = np.pi/180

# def data(msg):
#     print("msg.x",msg.x)
#     if 1<msg.x<10 and 1 < msg.y < 10:
#         cmd_vel.linear.x = 1
#         cmd_vel.angular.z = 0
#     else:
#         cmd_vel.angular.z = degree*90
#     #print("msg.y",msg.y)
#     #print("msg.theta",msg.theta)
#     # cmd_vel.linear.x = msg.data / 10
#     # cmd_vel.angular.z = msg.data % 10
#     pub.publish(cmd_vel)
#     pass

# sub = rospy.Subscriber('/turtle1/pose',Pose, callback = data)
# pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size= 1)
# rospy.spin()

####### simulator 사용 #######

import rospy
import numpy as np
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import LaserScan

rospy.init_node("sub_test")

rate = rospy.Rate(0.5)

speed_accel = Float64()
angle = Float64()

angle = 0  # -19.5 ~ 19.5 입력하면 됨 시뮬 자동차랑 각도 맞춤
angle = ( angle + 19.5) / 39 

speed = 2.26 # -2.26~2.26 입력하면 됨 
speed_pub = speed * 1404.7027199 - 134.223278
# 소수점 6자리를 구했으나 정확한 값이 아니라 오차가 있음

# if speed >= 1.3 :
#     #speed = (1062.333479 * speed) + 88.278849              # x가 커질수록 정확해짐
#     speed = (speed - 88.278849) / 1062.333479
#     0.000941324x -0.830990 = y
# elif 0.3 <= speed < 1.3:
#     #speed = (1164.251442*speed) -28.85558                   # x가 작을수록 정확해짐
#     speed = (speed - 0.024786) / 0.0008589 

roi_degree_offset = 35
def data(msg):
    
    #pub.publish(speed)
    #pub_angle.publish(angle)
    #print("angle : ", angle)
    

    ########## lidar ##########
    angle_min_degree = msg.angle_min / np.pi * 180
    angle_max_degree = msg.angle_max / np.pi * 180    
    '''
    원하는 각도를 입력해서 라이다의 각도랑 맞출려면 
    angle_radian = angle_degree *(np.pi/180)
    원하는 라디안을 입력해서 각도를 나타내려면 
    angle_degree = angle_radian * (180/np.pi)
    '''
    #print("angle_min_degree : {}, angle_max_degree : {} ".format(angle_min_degree,angle_max_degree))

    #angle_increment_degree = msg.angle_increment / np.pi * 180
    #print("angle_increment : ", angle_increment_degree)
    
    #lidar_len = len(msg.ranges)
    #print("갯수 : ",lidar_len)
    roi_ranges_left = list(msg.ranges[0:roi_degree_offset])       # 앞에 0도 기준 왼쪽으로 10도
    roi_ranges_right = list(msg.ranges[359-roi_degree_offset:359])   # 앞에 0도 기준 오른쪽으로 10도
    roi_ranges_right = roi_ranges_right[::-1]
    
    #print(roi_degree_right)
    #for i in range(0,10):
    #    print("왼쪽  {}번째 인덱스 lidar값: {}".format(i,roi_ranges_left[i]))
    #    print("오른쪽 {}번째 인덱스 lidar값: {}".format(i,roi_ranges_right[i]))
    #    #count += 1
    #print("다음 바퀴")
    #print(len(roi_degree_right))

    #print(roi_degree)


    ########## lidar를 통해 간단하게 장애물 만나면 정지 없으면 직진하는 코드 짜기 ##########
    count = 0
    for i in range(0,10):
        if roi_ranges_left[i] < 1.5:
            count += 1
        if roi_ranges_right[i] < 1.5:
            count += 1

    if count > 7:
        speed = 0.0
        speed_pub = 1062.333479 * speed
        #speed_pub = 0
    elif count <= 7:
        speed = 1.8
        speed_pub = 1062.333479 * speed 
        #speed_pub = 1000
        
    pub.publish(speed_pub)
    #print("speed : {}, car.velocity.x : {} ".format(speed, msg.velocity.x))


    #rate.sleep()

def velo(msg):
    #speed = 0.3
    #speed = 1062.333479 * speed
    vel_x = msg.velocity.x 
    print("speed :{}  velo.x : {}".format(speed_pub,vel_x))
    #pub.publish(speed)


pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
pub_angle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)
sub_Ego_topic = rospy.Subscriber('/Ego_topic',EgoVehicleStatus,callback = velo)
sub_lidar2D = rospy.Subscriber('/lidar2D',LaserScan,callback = data)

rospy.spin()
