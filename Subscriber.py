#! /usr/bin/env python3

# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
rospy.init_node('sub_test')
cmd_vel = Twist()

degree = np.pi/180

def data(msg):
    print("msg.x",msg.x)
    if 1<msg.x<10 and 1 < msg.y < 10:
        cmd_vel.linear.x = 1
        cmd_vel.angular.z = 0
    else:
        cmd_vel.angular.z = degree*90
    #print("msg.y",msg.y)
    #print("msg.theta",msg.theta)
    # cmd_vel.linear.x = msg.data / 10
    # cmd_vel.angular.z = msg.data % 10
    pub.publish(cmd_vel)
    pass

sub = rospy.Subscriber('/turtle1/pose',Pose, callback = data)
pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size= 1)
rospy.spin()
