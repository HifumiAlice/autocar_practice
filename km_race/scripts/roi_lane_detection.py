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

class roiLaneDetection :

    def __init__(self) :
        rospy.loginfo("[roi_lane_detection] publish(/roidist)")
        
        self.roi_left_x = 0
        self.roi_right_x = 640

        self.roi_upper_y = 300
        self.roi_lower_y = 480
        self.crop_image_y_size = self.roi_lower_y - self.roi_upper_y

        self.polygon_top_left_space = 160
        self.polygon_top_right_space = 180
        
        self.crop_image_size = 300
        self.thresh_value = 170

        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.roiLaneDetectioncallback )
        self.lane_moment_pub = rospy.Publisher("roidist", dist, queue_size=3)
        
        self.bridge = CvBridge()

    # run command : rosrun km_race  img_Processing R 150  <-- 150 is ignored.
    def roiLaneDetectioncallback(self, _data) :
        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)
        
        # create a zero array
        stencil = np.zeros_like(cv_image[:,:,0])

        # specify coordinates of the polygon
        # polygon = np.array([[0,470], [140,360], [500,360], [640,470]])
        rect_area = np.array([[self.roi_left_x, self.roi_lower_y], [self.roi_left_x, self.roi_upper_y], 
                            [self.roi_right_x, self.roi_upper_y], [self.roi_right_x, self.roi_lower_y]])

        polygon = np.array([[self.roi_left_x,self.roi_lower_y], [self.roi_left_x + self.polygon_top_left_space,self.roi_upper_y], 
                            [self.roi_right_x-self.polygon_top_right_space,self.roi_upper_y], [self.roi_right_x,self.roi_lower_y]])

        # fill polygon with ones
        cv2.fillConvexPoly(stencil, polygon, 1)

        polygone_img = cv2.bitwise_and(cv_image[:,:,0], cv_image[:,:,0], mask=stencil)

        ret, thresh = cv2.threshold(polygone_img, self.thresh_value, 255, cv2.THRESH_BINARY)

        # cv2.HoughLinesP(image, rho, theta, threshold, minLineLength, maxLineGap)
        lines = cv2.HoughLinesP(thresh, 1, np.pi/360, 100, maxLineGap=10)

        # create a copy of the original frame
        line_only_image = np.zeros_like(cv_image[:,:,0])

        # draw Hough lines
        if lines is not None :
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_only_image, (x1, y1), (x2, y2), (255, 0, 0), 3)

        # crop_image_left = line_only_image.copy()[350:480, 0:320]
        crop_image_left = line_only_image.copy()[self.roi_upper_y:self.roi_lower_y, 0:self.crop_image_size]
        
        countr = 0
        countl = 0

        self.dlflag = True
        ML = cv2.moments(crop_image_left)

        if ( ML['m00'] > 0 ) :
            self.xl = int(ML['m10']/ML['m00'])
            self.yl = int(ML['m01']/ML['m00'])

            for j in range(3) :
                for m in range(3) :
                    if ( self.yl + m >= self.crop_image_y_size ) or (self.yl - m < 0) :
                        continue

                    if (self.xl + j >= self.crop_image_size) or (self.xl - j < 0) :
                        continue

                    if (crop_image_left[self.yl + m, self.xl + j] == 255) :
                        countl += 1

                    if (crop_image_left[self.yl - m, self.xl - j] == 255) :
                        countl += 1

            # rospy.loginfo("xl={}, yl={}".format(self.xl, self.yl))
            cv2.circle(crop_image_left, (self.xl, self.yl), 3, (0, 255, 0), -1)
            cv2.circle(polygone_img, (self.xl, self.yl + self.roi_upper_y), 3, (0, 255, 0), -1)
        elif ML['m00'] == 0 :
            self.xl = 0
            self.yl = 0
            self.drflag = False

        # crop_image_right = line_only_image.copy()[350:480, 320:640]
        crop_image_right = line_only_image.copy()[self.roi_upper_y:self.roi_lower_y, (640-self.crop_image_size):640]
        self.drflag = True
        MR = cv2.moments(crop_image_right)
        if ( MR['m00'] > 0 ) :
            self.xr = int(MR['m10']/MR['m00'])
            self.yr = int(MR['m01']/MR['m00'])

            for j in range(3) :
                for m in range(3) :
                    if ( self.yr + m >= self.crop_image_y_size ) or (self.yr - m < 0) :
                        continue

                    if (self.xr + j >= self.crop_image_size) or (self.xr - j < 0) :
                        continue

                    if (crop_image_right[self.yr + m, self.xr + j] == 255) :
                        countr += 1

                    if (crop_image_right[self.yr - m, self.xr - j] == 255) :
                        countr += 1

            # rospy.loginfo("xr={}, yr={}".format(self.xr, self.yr))
            cv2.circle(crop_image_right, (self.xr, self.yr), 3, (0, 255, 0), -1)
            cv2.circle(polygone_img, (self.xr + self.roi_right_x - self.crop_image_size, self.yr+self.roi_upper_y), 3, (0, 255, 0), -1)
        elif MR['m00'] == 0 :
            self.xr = 0
            self.yr = 0
            self.drflag = False

        # rospy.loginfo("ROI-countr({}), countl({})".format(countr, countl))

        if countr != 18 :
            self.xr = 0
            self.yr = 0

        if countl != 18 :
            self.xl = 0
            self.yl = 0

        pub_data = dist()
        pub_data.dist_x_left = self.xl
        pub_data.dist_y_left = self.yl
        pub_data.dist_x_right = self.xr
        pub_data.dist_y_right = self.yr

        self.lane_moment_pub.publish(pub_data)

        cv2.imshow("ROI-poly gone image", polygone_img)
        # cv2.imshow("ROI-Image Threshholding", thresh)
        cv2.imshow("ROI-crop_image_left", crop_image_left)
        cv2.imshow("ROI-crop_image_right", crop_image_right)
        # cv2.imshow("line_only_image", line_only_image)

        cv2.waitKey(1)

def run( ):
    rospy.init_node('ROI_Lane_Detection', anonymous=True)
    roiLaneDetection( )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Program down")


if __name__ == "__main__":
    run( )
