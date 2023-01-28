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

class birdEyeView( ) :
    def __init__(self, _mode, _lowV) :
        rospy.loginfo("Bird Eye View")
        self.mode = _mode

        if int(_lowV) == 0 :
            self.lowV = 150
        else :
            self.lowV = int(_lowV)

        self.lowH = 0
        self.lowS = 0

        self.highH = 255
        self.highS = 255
        self.highV = 255

        self.thresh_value = 170

        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.birdEyeViewCallback )

        self.lane_moment_pub = rospy.Publisher("dist_x", dist, queue_size=3)
        
        self.initialized = False
        self.bridge = CvBridge()

    def birdEyeViewCallback(self, _data) :
        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)
        cv2.imshow("Input Image", cv_image)

        crop_image = cv_image.copy()[360:480, 0:640]  # 좌 (150,0), 우 (640-150-20, 0)=(170,0)
        # cv2.circle(crop_image, (150, 3), 3, (0, 255, 0), -1)
        # cv2.circle(crop_image, (470, 3), 3, (255, 0, 0), -1)
        # cv2.circle(crop_image, (5, 117), 3, (255, 0, 0), -1)
        # cv2.circle(crop_image, (615, 117), 3, (255, 0, 0), -1)

        # crop_image = cv_image.copy()[300:480, 0:640]  # 좌 (200,0), 우 (640-220, 0)=(420,0)
        # crop_image = cv_image.copy()[260:440, 0:640]  # 좌 (200,0), 우 (640-220, 0)=(420,0)
        
        # cv2.circle(crop_image, (290, 3), 3, (0, 255, 0), -1)
        # cv2.circle(crop_image, (345, 3), 3, (255, 0, 0), -1)
        # cv2.circle(crop_image, (50, 177), 3, (255, 0, 0), -1)
        # cv2.circle(crop_image, (580, 177), 3, (255, 0, 0), -1)

        # crop_image = cv_image.copy()[320:440, 0:640]    # 좌 160 -> 200, 우 160 -> 200+20(640-220=420)
        # cv2.circle(crop_image, (200, 3), 3, (0, 255, 0), -1)
        # cv2.circle(crop_image, (420, 3), 3, (255, 0, 0), -1)

        cv2.imshow("crop Image", crop_image)

        # Bird View Transformation
        height, width, ch = crop_image.shape
        # rospy.loginfo("height={}, width={}, channel={}".format(height, width, ch))  # height=480, width=640, ch=3
        # rospy.loginfo("shape={}".format(crop_image.shape))  # 


        # src 좌표점 : 원본의 좌표 (좌상, 좌하, 우상, 우하)
        # dst 좌표점 : 이동할 위치의 좌표
        # For Y = 360:480
        # src = np.float32([[150,0], [0, height], [470, 0], [width, height]])
        # dst = np.float32([[0,0], [0, height], [615, 0], [width, height]])
        # For Y = 300:480
        # src = np.float32([[240,0], [0, height], [390, 0], [620, height]])
        # dst = np.float32([[0,0], [0, height], [625, 0], [620, height]])
        # inv = np.float32([[0,0], [240, height], [640, 0], [390, height]])
        src = np.float32([[240,0], [0, height], [390, 0], [620, height]])
        dst = np.float32([[0,0], [0, height], [625, 0], [620, height]])
        inv = np.float32([[0,0], [240, height], [640, 0], [390, height]])

        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, inv)

        wraped_image = cv2.warpPerspective(crop_image, M, (width, height))  # Image wraping
        inv_image = cv2.warpPerspective(crop_image, Minv, (width, height))  # Image wraping
        cv2.imshow("wraped Image", wraped_image)
        cv2.imshow("inv Image", inv_image)

        #===== end of bird's eye view transformation ====================

        stencil = np.zeros_like(wraped_image[:,:,0])

        # specify coordinates of the polygon
        polygon = np.array([[0,180], [0,0], [640,0], [640,180]])

        # fill polygon with ones
        cv2.fillConvexPoly(stencil, polygon, 1)

        polygone_img = cv2.bitwise_and(wraped_image[:,:,0], wraped_image[:,:,0], mask=stencil)
        cv2.imshow("poly gone image", polygone_img)

        ret, thresh = cv2.threshold(polygone_img, self.thresh_value, 255, cv2.THRESH_BINARY)
        # cv2.imshow("Image Threshholding", thresh)

        lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 100, maxLineGap=200)

        # create a copy of the original frame
        # dmy = cv_image[:,:,0].copy()

        line_only_image = np.zeros_like(wraped_image[:,:,0])

        # draw Hough lines
        if lines is not None :
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # cv2.line(dmy, (x1, y1), (x2, y2), (255, 0, 0), 3)
                cv2.line(line_only_image, (x1, y1), (x2, y2), (255, 0, 0), 3)

        # size of line_only_image : [180:640]
        crop_image_left = line_only_image.copy()[0:180, 0:320]
        self.dlflag = True
        ML = cv2.moments(crop_image_left)
        self.xl = 0
        self.yl = 0
        if ( ML['m00'] > 0 ) :
            self.xl = int(ML['m10']/ML['m00'])
            self.yl = int(ML['m01']/ML['m00'])
            # rospy.loginfo("xl={}, yl={}".format(self.xl, self.yl))
            cv2.circle(crop_image_left, (self.xl, self.yl), 3, (0, 255, 0), -1)
            cv2.circle(line_only_image, (self.xl, self.yl), 3, (0, 255, 0), -1)
        elif ML['m00'] == 0 :
            self.drflag = False

        crop_image_right = line_only_image.copy()[0:180, 321:640]
        self.drflag = True
        MR = cv2.moments(crop_image_right)
        self.xr = 0
        self.yr = 0
        if ( MR['m00'] > 0 ) :
            self.xr = int(MR['m10']/MR['m00'])
            self.yr = int(MR['m01']/MR['m00'])
            # rospy.loginfo("xr={}, yr={}".format(self.xr, self.yr))
            cv2.circle(crop_image_right, (self.xr, self.yr), 3, (0, 255, 0), -1)
            cv2.circle(line_only_image, (self.xr+320, self.yr), 3, (0, 255, 0), -1)
        elif MR['m00'] == 0 :
            self.drflag = False

        dist_l = 320 - self.xl          # 중심으로부터 왼쪽 차선까지의 거리
        dist_r = self.xr                # 중심으로부터 오른쪽 차선까지의 거리

        pub_data = dist()
        pub_data.dist_left = self.xl
        pub_data.dist_right = self.xr
        self.lane_moment_pub.publish(pub_data)

        cv2.imshow("crop_image_left", crop_image_left)
        cv2.imshow("crop_image_right", crop_image_right)
        cv2.imshow("line_only_image", line_only_image)

        if (dist_l > dist_r) :
            # 왼쪽으로 steering 조정
            pass
        else :
            # 오른쪽으로 steering 조정
            pass

        cv2.waitKey(1)

def run( ):
    rospy.init_node('bird_eye_view', anonymous=True)
    ip = birdEyeView( )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Program down")


if __name__ == "__main__":
    run( )
