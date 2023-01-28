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

class img_Processing :
    def __init__(self, _mode, _lowV) :
        rospy.loginfo("Image_Processing - mode({}), lowV({})".format(_mode, _lowV))
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

        if _mode == 'C' :           # For Check Image of the detected lane
            rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.imgCheckcallback )
        elif _mode == 'M' :
            rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.imgMomentcallback )
        elif _mode == 'R' :
            rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.roiLaneDetectioncallback )
        elif _mode == 'B' :
            rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.birdEyeViewCallback )

        self.lane_moment_pub = rospy.Publisher("dist_x", dist, queue_size=3)
        
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
        # rospy.loginfo(cv_image.shape)
        # rospy.loginfo(type(cv_image))

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        crop_image_left = cv_image.copy()[300:480, 0:320, :]
        crop_image_right = cv_image.copy()[300:480, 320:640, :]

        # cv2.line(crop_image, (320, 300), (320,480), (255, 0, 0), 1)
        
        low_H = cv2.getTrackbarPos('low_H', 'Rect_Image')
        low_S = cv2.getTrackbarPos('low_S', 'Rect_Image')
        low_V = cv2.getTrackbarPos('low_V', 'Rect_Image')
        high_H = cv2.getTrackbarPos('high_H', 'Rect_Image')
        high_S = cv2.getTrackbarPos('high_S', 'Rect_Image')
        high_V = cv2.getTrackbarPos('high_V', 'Rect_Image')

        lower_lane = np.array([low_H, low_S, low_V])
        upper_lane = np.array([high_H, high_S, high_V])

        lane_image_left = cv2.inRange( crop_image_left, lower_lane, upper_lane)
        lane_image_right = cv2.inRange( crop_image_right, lower_lane, upper_lane)

        cv2.imshow("Lane Image Left", lane_image_left)
        cv2.imshow("Lane Image Right", lane_image_right)

        # ch1, ch2, ch3 = cv2.split(cv_image)
        # cv2.imshow("Rect_Image", cv_image)
        # cv2.imshow("Rect_Image - CH1", ch1) # 색상
        # cv2.imshow("Rect_Image - CH2", ch2) # Saturation
        # cv2.imshow("Rect_Image - CH3", ch3) # 밝기

        cv2.waitKey(1)

    # run command : rosrun km_race img_Processing.py M xxx  <-- xxx should be determinded by imgCheckcallback 
    def imgMomentcallback(self, _data) :
        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        cv2.imshow("CV Image", cv_image)


        # crop_image_left = cv_image.copy()[400:480, 0:250, :]
        # crop_image_right = cv_image.copy()[400:480, 390:640, :]
        
        crop_image_left = cv_image.copy()[300:480, 0:320, :]
        crop_image_right = cv_image.copy()[300:480, 321:640, :]

        lower_lane = np.array([self.lowH, self.lowS, self.lowV])
        upper_lane = np.array([self.highH, self.highS, self.highV])

        lane_image_left = cv2.inRange( crop_image_left, lower_lane, upper_lane)
        lane_image_right = cv2.inRange( crop_image_right, lower_lane, upper_lane)

        self.dlflag = True
        ML = cv2.moments(lane_image_left)
        if ( ML['m00'] > 0 ) :
            self.xl = int(ML['m10']/ML['m00'])
            self.yl = int(ML['m01']/ML['m00'])
            cv2.circle(crop_image_left, (self.xl, self.yl), 3, (0, 255, 0), -1)
        elif ML['m00'] == 0 :
            self.dlflag = False

        cv2.imshow("Result Image Left", crop_image_left)

        self.drflag = True
        MR = cv2.moments(lane_image_right)
        if ( MR['m00'] > 0 ) :
            self.xr = int(MR['m10']/MR['m00'])
            self.yr = int(MR['m01']/MR['m00'])
            cv2.circle(crop_image_right, (self.xr, self.yr), 3, (0, 255, 0), -1)
        elif MR['m00'] == 0 :
            self.drflag = False

        pub_data = dist()
        pub_data.dist_left = self.xl
        pub_data.dist_right = self.xr

        self.lane_moment_pub.publish(pub_data)

        cv2.imshow("Result Image Right", crop_image_right)

        # cv2.imshow("Lane Image Left", lane_image_left)
        # cv2.imshow("Lane Image Right", lane_image_right)

        cv2.waitKey(1)

    # run command : rosrun km_race  img_Processing R 150  <-- 150 is ignored.
    def roiLaneDetectioncallback(self, _data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)
        
        # create a zero array
        stencil = np.zeros_like(cv_image[:,:,0])

        # specify coordinates of the polygon
        # polygon = np.array([[0,470], [140,360], [500,360], [640,470]])
        polygon = np.array([[0,480], [0,280], [640,280], [640,480]])

        # fill polygon with ones
        cv2.fillConvexPoly(stencil, polygon, 1)

        polygone_img = cv2.bitwise_and(cv_image[:,:,0], cv_image[:,:,0], mask=stencil)
        cv2.imshow("poly gone image", polygone_img)

        ret, thresh = cv2.threshold(polygone_img, self.thresh_value, 255, cv2.THRESH_BINARY)
        cv2.imshow("Image Threshholding", thresh)

        lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 30, maxLineGap=200)

        # create a copy of the original frame
        # dmy = cv_image[:,:,0].copy()

        line_only_image = np.zeros_like(cv_image[:,:,0])

        # draw Hough lines
        if lines is not None :
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # cv2.line(dmy, (x1, y1), (x2, y2), (255, 0, 0), 3)
                cv2.line(line_only_image, (x1, y1), (x2, y2), (255, 0, 0), 3)

        # cv2.imshow("Hough Lines", dmy)

        # rospy.loginfo("type of dmy = {}".format(type(dmy)))
        # rospy.loginfo("shape of dmy = {}".format(dmy.shape))

        # crop_image_left = line_only_image.copy()[350:480, 0:250]
        crop_image_left = line_only_image.copy()[350:480, 0:320]
        self.dlflag = True
        ML = cv2.moments(crop_image_left)
        self.xl = 0
        self.yl = 0
        if ( ML['m00'] > 0 ) :
            self.xl = int(ML['m10']/ML['m00'])
            self.yl = int(ML['m01']/ML['m00'])
            # rospy.loginfo("xl={}, yl={}".format(self.xl, self.yl))
            cv2.circle(crop_image_left, (self.xl, self.yl), 3, (0, 255, 0), -1)
        elif ML['m00'] == 0 :
            self.drflag = False

        # crop_image_right = line_only_image.copy()[350:480, 390:640]
        crop_image_right = line_only_image.copy()[350:480, 321:640]
        self.drflag = True
        MR = cv2.moments(crop_image_right)
        self.xr = 0
        self.yr = 0
        if ( MR['m00'] > 0 ) :
            self.xr = int(MR['m10']/MR['m00'])
            self.yr = int(MR['m01']/MR['m00'])
            # rospy.loginfo("xr={}, yr={}".format(self.xr, self.yr))
            cv2.circle(crop_image_right, (self.xr, self.yr), 3, (0, 255, 0), -1)
        elif MR['m00'] == 0 :
            self.drflag = False

        pub_data = dist()
        pub_data.dist_left = self.xl
        pub_data.dist_right = self.xr
        self.lane_moment_pub.publish(pub_data)

        cv2.imshow("crop_image_left", crop_image_left)
        cv2.imshow("crop_image_right", crop_image_right)
        # cv2.imshow("line_only_image", line_only_image)

        cv2.waitKey(1)

    def birdEyeViewCallback(self, _data) :
        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)
        cv2.imshow("Input Image", cv_image)

        # crop_image = cv_image.copy()[360:480, 0:640]  # 좌 (150,0), 우 (640-150-20, 0)=(170,0)
        # cv2.circle(crop_image, (150, 3), 3, (0, 255, 0), -1)
        # cv2.circle(crop_image, (470, 3), 3, (255, 0, 0), -1)
        # cv2.circle(crop_image, (5, 117), 3, (255, 0, 0), -1)
        # cv2.circle(crop_image, (615, 117), 3, (255, 0, 0), -1)

        # crop_image = cv_image.copy()[300:480, 0:640]  # 좌 (200,0), 우 (640-220, 0)=(420,0)
        crop_image = cv_image.copy()[260:440, 0:640]  # 좌 (200,0), 우 (640-220, 0)=(420,0)
        
        cv2.circle(crop_image, (290, 3), 3, (0, 255, 0), -1)
        cv2.circle(crop_image, (345, 3), 3, (255, 0, 0), -1)
        cv2.circle(crop_image, (50, 177), 3, (255, 0, 0), -1)
        cv2.circle(crop_image, (580, 177), 3, (255, 0, 0), -1)

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
        src = np.float32([[290,0], [50, height], [345, 0], [580, height]])
        dst = np.float32([[50,0], [50, height], [580, 0], [580, height]])
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

    def findLane(self, _data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)
        cv2.imshow("FindLane Input Image", cv_image)
        crop_image = cv_image.copy()[260:340, 0:640]  
        cv2.imshow("FindLane crop Image", crop_image)


 
def nothing(pos):
    pass

def run(_mode, _lowV=150):
    rospy.init_node('image_processing', anonymous=True)
    ip = img_Processing(_mode, _lowV)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Program down")


if __name__ == "__main__":
    if len(sys.argv) == 3 :
        run(sys.argv[1], sys.argv[2])
    else :
        run(sys.argv[1])
