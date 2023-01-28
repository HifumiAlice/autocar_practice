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

class checkCVT :
    def __init__(self) :
        rospy.loginfo("Check CVT")
        self.lowH = 0
        self.lowS = 0
        self.lowV = 150

        self.highH = 255
        self.highS = 255
        self.highV = 255

        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.imgCheckcallback )

        self.cvtvalue_pub = rospy.Publisher("cvt_value", Int32, queue_size=3)

        self.initialized = False
        self.bridge = CvBridge()


    # run command : rosrun km_race img_Processing.py M xxx
    def imgCheckcallback(self, _data) :
        if (self.initialized == False) :
            cv2.namedWindow("Rect_Image", cv2.WINDOW_NORMAL)
            cv2.createTrackbar('low_H', 'Rect_Image', 0, 255, nothing)
            cv2.createTrackbar('low_S', 'Rect_Image', 0, 255, nothing)
            # Lane Image 확인하고, 값 변경, default=150
            cv2.createTrackbar('low_V', 'Rect_Image', self.lowV, 255, nothing)    
            cv2.createTrackbar('high_H', 'Rect_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Rect_Image', 255, 255, nothing)
            cv2.createTrackbar('high_V', 'Rect_Image', 255, 255, nothing)
            self.initialized = True

        # rospy.loginfo(len(_data.data))
        # rospy.loginfo(type(_data.data))

        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        crop_image_left = cv_image.copy()[300:480, 0:320, :]
        crop_image_right = cv_image.copy()[300:480, 320:640, :]

        # cv2.line(crop_image, (320, 300), (320,480), (255, 0, 0), 1)
        
        self.lowH = cv2.getTrackbarPos('low_H', 'Rect_Image')
        self.lowS = cv2.getTrackbarPos('low_S', 'Rect_Image')
        self.lowV = cv2.getTrackbarPos('low_V', 'Rect_Image')
        self.highH = cv2.getTrackbarPos('high_H', 'Rect_Image')
        self.highS = cv2.getTrackbarPos('high_S', 'Rect_Image')
        self.highV = cv2.getTrackbarPos('high_V', 'Rect_Image')

        lower_lane = np.array([self.lowH, self.lowS, self.lowV])
        upper_lane = np.array([self.highH, self.highS, self.highV])

        lane_image_left = cv2.inRange( crop_image_left, lower_lane, upper_lane)
        lane_image_right = cv2.inRange( crop_image_right, lower_lane, upper_lane)

        self.cvtvalue_pub.publish(Int32(self.lowV))
        # rospy.loginfo("lowV = {}, type={}".format(self.lowV, type(self.lowV)))

        cv2.imshow("Lane Image Left", lane_image_left)
        cv2.imshow("Lane Image Right", lane_image_right)

        # ch1, ch2, ch3 = cv2.split(cv_image)
        # cv2.imshow("Rect_Image", cv_image)
        # cv2.imshow("Rect_Image - CH1", ch1) # 색상
        # cv2.imshow("Rect_Image - CH2", ch2) # Saturation
        # cv2.imshow("Rect_Image - CH3", ch3) # 밝기

        cv2.waitKey(1)
 
def nothing(pos):
    pass

def run( ):
    rospy.init_node('check_cvt', anonymous=True)
    checkCVT( )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Program down")


if __name__ == "__main__":
    run( )
