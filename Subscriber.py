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
x = -2.26 # -2.26~2.26
#speed_km = (1062.333479 * x) + 88.278849              # x가 커질수록 정확해짐
speed_km = (1164.251442*x) -28.85558                   # x가 작을수록 정확해짐
# 소수점 6자리를 구했으나 정확한 값이 아니라 오차가 있음

def data(msg):
    #print("EgoVehicleStatus : ",msg)       
    #print("car.velocity.x : ", msg.velocity.x) 

    # if msg.velocity.x < 1:
    #     pub.publish(speed_km)
    #     #pub.publish(1170)

    #     #speed += 10
    # elif msg.velocity.x == 1:        
    #     print("그만")
    # elif msg.velocity.x > 1 :
    #     print("허걱")
    pub.publish(speed_km)
    print("speed : {}, car.velocity.x : {} ".format(speed_km, msg.velocity.x))
    pub_angle.publish(0.5)

    #rate.sleep()

    

pub = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
pub_angle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)
sub = rospy.Subscriber('/Ego_topic',EgoVehicleStatus,callback = data)

rospy.spin()