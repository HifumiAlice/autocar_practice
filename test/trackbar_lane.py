#!/usr/bin/env python3

# -*- coding : utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class Cam_Trackbar():

    def __init__(self):
        rospy.init_node("Cam_trackbar")
        self.sub = rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.callback)
        self.cvbridge = CvBridge()
        
        self.count = 0
        
        rospy.on_shutdown(self.TrackShutdown)
        rospy.spin()
        pass

    def callback(self,data):
        try:
            cv_image = self.cvbridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Convertion Error!")
            print(e)
        
        if self.count == 0:
            cv2.namedWindow("Trackbar",cv2.WINDOW_NORMAL)
            cv2.createTrackbar("Cthr1","Trackbar",100,255,nothing)
            cv2.createTrackbar("Cthr2","Trackbar",150,255,nothing)
            self.count += 1
        can_thr1 = cv2.getTrackbarPos('Cthr1',"Trackbar")
        can_thr2 = cv2.getTrackbarPos('Cthr2',"Trackbar")

        print(can_thr2)
        image_gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        image_blur = cv2.GaussianBlur(image_gray,(5,5),0)
        image_canny = cv2.Canny(np.uint8(image_blur),can_thr1,can_thr2)
        
        
        cv2.imshow("original",cv_image)
        cv2.imshow("image_canny",image_canny)
        cv2.waitKey(1)


    def TrackShutdown(self):
        print("I am dead!!")

def nothing(pos):
    pass
if __name__ == "__main__":
    Trackbar = Cam_Trackbar()