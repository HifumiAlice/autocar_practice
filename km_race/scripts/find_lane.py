#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import sys

from std_msgs.msg import String
from std_msgs.msg import Int32
from km_race.msg import dist

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class findLane :
    def __init__(self) :
        self.thresh_value = 170

        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.callback )
        self.lane_moment_pub = rospy.Publisher("dist_x", dist, queue_size=3)
        
        self.initialized = False
        self.bridge = CvBridge()

    def callback(self, _data) :
        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)
        cv2.imshow("FindLane Input Image", cv_image)
        crop_image = cv_image.copy()[260:340, 0:640]  
        cv2.imshow("FindLane crop Image", crop_image)


 
def nothing(pos):
    pass

def run(_):
    rospy.init_node('Find Lanes', anonymous=True)
    ip = findLane( )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Program down")


if __name__ == "__main__":
    run( )  