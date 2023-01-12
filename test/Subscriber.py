#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


def callback(data):
    print(data.data)
    new_msg = (Twist())
    new_msg.linear.x = data.data % 5
    new_msg.angular.z = 3
    pub.publish(new_msg)

rospy.init_node('Subsciber')
pub = rospy.Publisher('/turtle1/cmd_vel',Twist)
sub = rospy.Subscriber('counter',Int32,callback)

rospy.spin()