#! /usr/bin/env python3

# -*- coding : utf-8 -*-

import rospy
from std_msgs.msg import Int32

rospy.init_node("pub_test")  # 1. init node 설정
pub = rospy.Publisher('counter',Int32,queue_size=1) # 2. node가 publisher인지 subscriber인지 결정

rate = rospy.Rate(2)

count = 0

while not rospy.is_shutdown():
    count = count + 1
    pub.publish(count) # 3. publisher가 publish 하는 것