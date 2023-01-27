#! /usr/bin/env python3

from turtle import color
import rospy

import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class camera_sim_light():
    def __init__(self):
        rospy.init_node('image_to_receiver', anonymous=False)
        self.pubcam1 = rospy.Publisher("/pub_img_light1", Image, queue_size=10)
        self.pubcam2 = rospy.Publisher("/pub_img_light", Image, queue_size=10)        
        self.subcam = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.bridge = CvBridge()
        rospy.on_shutdown(self.cam_shutdown)
        rospy.spin()


    def callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("convertion error")
            print(e)
        height, width, channel = cv_image.shape
        cvt_hls_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS)
        cvt_hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        zero_img = np.zeros_like(cv_image)
        region_mask_up = cv2.rectangle(zero_img, (0,0), (width, int(height*25/100)), (255, 255, 255), -1)
        masked_up_img = cv2.bitwise_and(cvt_hsv_img, region_mask_up)

        # color check
        lower_r = (0, 80, 80)
        upper_r = (0, 255, 255)
        

        lower_g = (60-10, 30, 30)
        upper_g = (60+10, 255, 255)


        img_color_check1 = cv2.inRange(masked_up_img, lower_g, upper_g)
        img_color_check2 = cv2.inRange(masked_up_img, lower_r, upper_r)
        
        img_color_result_1 = cv2.bitwise_and(masked_up_img, masked_up_img, mask=img_color_check1)
        img_color_result_2 = cv2.bitwise_not( img_color_result_1,img_color_result_1, mask = img_color_check2)
 

        
        color_check = img_color_result_2.mean()
        print((img_color_result_2.mean()))
        #print(img_color_result_1.mean)
        if 0.3 < color_check < 0.5:
            print("left-go!!")
        else:
            print("stop!!")

        try:    
            self.pubcam2.publish(self.bridge.cv2_to_imgmsg(img_color_result_2, "rgb8"))
            self.pubcam1.publish(self.bridge.cv2_to_imgmsg(img_color_result_1, "rgb8"))
        except CvBridgeError as e:
            print("publish error")
            print(e)

        cv2.imshow("img_color_result_2",img_color_result_1)


    def cam_shutdown(self):
        print("I'm dead!")


if __name__ == "__main__":

    rp = camera_sim_light()

