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

class cvtImageAuto :
    def __init__(self) :
        rospy.loginfo("CVT Image")

        self.lowH = 0
        self.lowS = 0
        self.lowV = 150

        self.highH = 255
        self.highS = 255
        self.highV = 255

        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.imgMomentcallback )

        self.lane_moment_pub = rospy.Publisher("cvtdist", dist, queue_size=3)
        
        self.initialized = False
        self.lowVSearched = False

        self.bridge = CvBridge()

    # run command : rosrun km_race img_Processing.py M xxx  <-- xxx should be determinded by imgCheckcallback 
    def imgMomentcallback(self, _data) :
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

        cvt_left_x = 0
        cvt_right_x = 640
        cvt_center_x = 320
        cvt_upper_y = 300
        cvt_lower_y = 480

        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        if (self.lowVSearched) :
            cv2.setTrackbarPos('low_V', 'Rect_Image', self.lowV)
            self.lowVSearched = False

        # crop_image_left = cv_image.copy()[400:480, 0:250, :]
        # crop_image_right = cv_image.copy()[400:480, 390:640, :]
        
        self.crop_image_left = cv_image.copy()[cvt_upper_y:cvt_lower_y, cvt_left_x:cvt_center_x, :]
        self.crop_image_right = cv_image.copy()[cvt_upper_y:cvt_lower_y, cvt_center_x:cvt_right_x, :]

        self.lowH = cv2.getTrackbarPos('low_H', 'Rect_Image')
        self.lowS = cv2.getTrackbarPos('low_S', 'Rect_Image')
        self.lowV = cv2.getTrackbarPos('low_V', 'Rect_Image')
        self.highH = cv2.getTrackbarPos('high_H', 'Rect_Image')
        self.highS = cv2.getTrackbarPos('high_S', 'Rect_Image')
        self.highV = cv2.getTrackbarPos('high_V', 'Rect_Image')

        # rospy.loginfo("lowV = {}".format(self.lowV))

        lower_lane = np.array([self.lowH, self.lowS, self.lowV])
        upper_lane = np.array([self.highH, self.highS, self.highV])

        lane_image_left = cv2.inRange( self.crop_image_left, lower_lane, upper_lane)
        lane_image_right = cv2.inRange( self.crop_image_right, lower_lane, upper_lane)

        self.dlflag = True
        ML = cv2.moments(lane_image_left)
        if ( ML['m00'] > 0 ) :
            self.xl = int(ML['m10']/ML['m00'])
            self.yl = int(ML['m01']/ML['m00'])

            cv2.circle(lane_image_left, (self.xl, self.yl), 3, (0, 255, 0), -1)
            cv2.circle(self.crop_image_left, (self.xl, self.yl), 3, (0, 255, 0), -1)
            cv2.circle(cv_image, (self.xl, self.yl + cvt_upper_y), 3, (0, 255, 0), -1)
        elif ML['m00'] == 0 :
            self.xl = 319
            self.yl = 0
            self.dlflag = False

        self.drflag = True
        MR = cv2.moments(lane_image_right)
        if ( MR['m00'] > 0 ) :
            self.xr = int(MR['m10']/MR['m00'])
            self.yr = int(MR['m01']/MR['m00'])

            cv2.circle(lane_image_right, (self.xr, self.yr), 3, (0, 255, 0), -1)
            cv2.circle(self.crop_image_right, (self.xr, self.yr), 3, (0, 255, 0), -1)
            cv2.circle(cv_image, (self.xr+320, self.yr+cvt_upper_y), 3, (0, 255, 0), -1)
        elif MR['m00'] == 0 :
            self.xr = 0
            self.yr = 0
            self.drflag = False

        # check color of a moment point : if 255, it is ok, if not, there are some noise.
        # rospy.loginfo("color of lane_image_right[{}, {}] = {}".format(self.xr, self.yr, lane_image_right[self.yr, self.xr]))
        if (lane_image_left[self.yl, self.xl] == 0) or (lane_image_right[self.yr, self.xr] == 0) :
            self.lowV = self.searchLowV()
            rospy.loginfo("Find lowV = {}".format(self.lowV))

        pub_data = dist()
        pub_data.dist_x_left = self.xl
        pub_data.dist_y_left = self.yl
        pub_data.dist_x_right = self.xr
        pub_data.dist_y_right = self.yr
        self.lane_moment_pub.publish(pub_data)

        cv2.imshow("Rect_Image", cv_image)
        cv2.imshow("CVT-Result Image Left", self.crop_image_left)
        cv2.imshow("CVT-Result Image Right", self.crop_image_right)

        cv2.imshow("CVT-Lane Image Left", lane_image_left)
        cv2.imshow("CVT-Lane Image Right", lane_image_right)

        cv2.waitKey(1)

    def searchLowV(self) :
        mlowV = plowV = self.lowV
        
        for i in range(50) :
            mlowV = mlowV - 1
            if ( self.subSearchlowV(mlowV) != 0 ) :
                return mlowV
            plowV = plowV + 1
            if ( self.subSearchlowV(plowV) != 0 ) :
                return plowV

    def subSearchlowV(self, lowV) :
        lower_lane = np.array([self.lowH, self.lowS, lowV])
        upper_lane = np.array([self.highH, self.highS, self.highV])

        lane_image_left = cv2.inRange( self.crop_image_left, lower_lane, upper_lane)
        lane_image_right = cv2.inRange( self.crop_image_right, lower_lane, upper_lane)

        ML = cv2.moments(lane_image_left)
        if ( ML['m00'] > 0 ) :
            xl = int(ML['m10']/ML['m00'])
            yl = int(ML['m01']/ML['m00'])

        MR = cv2.moments(lane_image_right)
        if ( MR['m00'] > 0 ) :
            xr = int(MR['m10']/MR['m00'])
            yr = int(MR['m01']/MR['m00'])

        count = 0

        if (( ML['m00'] > 0 ) and ( MR['m00'] > 0 ) ) :
            for j in range(3) :
                for m in range(3) :
                    if (lane_image_right[yr + m, xr + j] == 0) :
                        count += 1

                    if (lane_image_right[yr - m, xr - j] == 0) :
                        count += 1

                    if (lane_image_left[yl + m, xl + j] == 0) :
                        count += 1

                    if (lane_image_left[yl - m, xl - j] == 0) :
                        count += 1

        if count == 0 :
            # rospy.loginfo("find lowV = {}".format(lowV))
            self.lowVSearched = True
            return lowV
        
        return 0

        
    def getLaneCoordinates(self, _data) :
        pass


def nothing(pos):
    pass

def run( ):
    rospy.init_node('cvtImage', anonymous=True)
    cvtImageAuto( )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Program down")


if __name__ == "__main__":
    run( )
