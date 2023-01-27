#! /usr/bin/env python3

# -*- coding : utf-8 -*-

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

rospy.init_node("pub_test")  # 1. init node 설정
#pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=1) # 2. node가 publisher인지 subscriber인지 결정
pub1 = rospy.Publisher('counter',Int32,queue_size= 10)
rate = rospy.Rate(1)

count = 0
msg = Twist()

msg.linear.x = 3.0
msg.linear.y = 0.0
msg.linear.z = 0.0

msg.angular.x = 0.0
msg.angular.y = 0.0
msg.angular.z = 1.0

while not rospy.is_shutdown():    
    count += 1
    #pub.publish(msg) # 3. publisher가 publish 하는 것
    pub1.publish(count)
    rate.sleep()