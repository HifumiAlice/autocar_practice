#!/usr/bin/env python
# -*- coding: utf-8 -*-

# https://github.com/mlsdpk/ros-lane-follower/search?l=python 
# Error ========================================

from tkinter import W
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from km_race.msg import dist


# LaneDetection Class for finding cross road error from the undistorted image
class thLaneDetection( ):
    def __init__(self):
        self.draw_img = False
        self.check_fps = True
        self.scale_percent = 40

        rospy.loginfo("Threshold Lane Detection")
        
        self.thresh_value = 170

        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.processImage )
        self.lane_moment_pub = rospy.Publisher("roidist_x", dist, queue_size=3)
        
        self.bridge = CvBridge()


    def processImage(self, _data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(_data)

        # Resized Image
        resized_img = self.resize_img(cv_image, self.scale_percent)

        # Thresholded Image
        thresh_img = self.findThreshold(resized_img)

        # Find cross track error and angle error
        cte, angle, output_image = self.calculateContours(thresh_img, resized_img)

        cv2.imshow("Inpur Image", cv_image)
        cv2.imshow("resized_img", resized_img)
        cv2.imshow("thresh_img", thresh_img)
        # cv2.imshow("output_image", output_image)

        cv2.waitKey(1)

    def resize_img(self, img, scale_percent):
        width = int(img.shape[1]*scale_percent/100)
        height = int(img.shape[0]*scale_percent/100)
        dim = (width, height)
        resized_img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

        resized_img = resized_img[75:,:]
        return resized_img

    def findThreshold(self, img):

        # Convert input BGR image to HSV color space and take S-Channel (Saturation)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        s_channel = hsv[:,:,1]

        # Remove noise using 3x3 Kernel (Gaussian Filter)
        blurred = cv2.GaussianBlur(s_channel, (3,3), 1)
        # cv2.imshow("blurred Image", blurred)

        ret, thresh = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY)
        # cv2.imshow("thresh Image", thresh)

        # Binarize the copy image with global thresholding
        # binarized_image = np.zeros_like(blurred)
        # binarized_image[blurred>120] = 1
        # cv2.imshow("binarized_image 2", binarized_image)

        # Morphological Transformations (Erosion & Dilation)
        kernel = np.ones((3,3), dtype=np.uint8)

        # dilated_img = cv2.dilate(binarized_image, kernel)
        dilated_img = cv2.dilate(thresh, kernel)
        # cv2.imshow("dilated_img Image", dilated_img)

        return dilated_img

    def calculateContours(self, thresh_img, original_img):

        img, contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # ---------------------------------------------------------------------------
        # Originally developed by OutOfTheBots
        if len(contours) > 0:
            blackbox = cv2.minAreaRect(contours[0])
            (x_min, y_min), (w_min, h_min), ang = blackbox
            if ang < -45: ang += 90
            if w_min < h_min and ang > 0: ang = (90-ang)*-1
            if w_min > h_min and ang < 0: ang = 90 + ang
            setpoint = thresh_img.shape[1]/2
            cte = -int(x_min - setpoint)
            angle = -int(ang)

            if self.draw_img:
                box = cv2.boxPoints(blackbox)
                box = np.int0(box)
                _ = cv2.drawContours(original_img, [box], 0, (255,0,0),1)
                ang_msg = "Angle Error = " + str(angle)
                err_msg = "Error = " + str(cte)
                cv2.putText(original_img, ang_msg, (130,25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,0), 1)
                cv2.putText(original_img, err_msg, (130,50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)
                cv2.line(original_img, (int(x_min), 0), (int(x_min), thresh_img.shape[0]), (0,0,255), 1)
        # ---------------------------------------------------------------------------
        else:
            cte = None
            angle = None

        return cte, angle, original_img

def run( ):
    rospy.init_node('Threshold_Lane_Detection', anonymous=True)
    thLaneDetection( )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Program down")


if __name__ == "__main__":
    run( )