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
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
rospy.init_node("sub_test")

rate = rospy.Rate(0.3)

speed_accel = Float64()
angle = Float64()

angle = 0  # -19.5 ~ 19.5 입력하면 됨 시뮬 자동차랑 각도 맞춤
angle = ( angle + 19.5) / 39 

speed = 1.0 # -2.26~2.26 입력하면 됨 
speed = (speed + 134.223278) / 1404.7027199
# 소수점 6자리를 구했으나 정확한 값이 아니라 오차가 있음

# if speed >= 1.3 :
#     #speed = (1062.333479 * speed) + 88.278849              # x가 커질수록 정확해짐
#     speed = (speed - 88.278849) / 1062.333479
#     0.000941324x -0.830990 = y
# elif 0.3 <= speed < 1.3:
#     #speed = (1164.251442*speed) -28.85558                   # x가 작을수록 정확해짐
#     speed = (speed - 0.024786) / 0.0008589 

def data(msg):
    
    pub.publish(speed)
    pub_angle.publish(angle)
    #print("angle : ", angle)
    print("speed : {}, car.velocity.x : {} ".format(speed, msg.velocity.x))
   

    #rate.sleep()

    

pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
pub_angle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)
sub = rospy.Subscriber('/Ego_topic',EgoVehicleStatus,callback = data)

rospy.spin()