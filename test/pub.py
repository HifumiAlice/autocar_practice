#! /usr/bin/env python3

# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64

rospy.init_node("pub_motor_speed")

pub_speed = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
pub_angle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)
count = 100
rate = rospy.Rate(2)

while not rospy.is_shutdown():
    count += 100
    pub_speed.publish(count)
    pub_angle.publish(0.5)
    rate.sleep()


'''
각도는 0이 -19.5도고 1이 19.5도이다 입력값을 x 시뮬 각도가 y라 하자
(0.-19.5) (1,19.5) 39 = y+19.5/x    0 <= x <= 1    -19.5 <= y <= 19.5
y = 39x - 19.5    x = ( y + 19.5) / 39


speed = 1000일때 accel 대략 0.858225 a,b  x가 speed y가 car.velocity.x
speed = 2000일때 accel 대략 1.799549 c,d
(y + 0.830990) / 0.000941324 = x

velocity.x = 0.45150 speed = 500
velocity.x = 2.231236 speed = 3000
(0.45150,500) (2.231236, 3000)       y + 134.223278 / 1404.7027199 = x
speed * 1404.7027199 - 134.223278


x = y(1062.333479) + 88.278849 원하는 velocity.x 값을 y에 넣으면 됨 넣으면 됨

(x - 88.278849) / 1062.333479 = y

speed : 2489.1525115399995 car.velocity.x : 2.16277  a,b
speed : 1150.612328, car.velocity.x : 1.01307        c,d

(2.16277,2489.152511) (1.01307,1150.612328)

(2489.152511-1150.612328)/2.16277-1.01307 = y-2489.152511/x-2.16277

(1338.539883)*(x-2.16277) = (y-2489.152511) * (1.1497)

1164.251442*x -28.85558 = y

1.1497*(x-c) = (y-d) * (1338.54018)
0.0008589x - 0.988284 = y - 1.01307
0.0008589x + 0.024786 = y

(y - 0.024786) / 0.0008589 = x


(0,0) (2.255433,3500)
3500/2.231236 = y/x
1551.808455405237x = y


'''