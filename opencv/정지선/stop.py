#! /usr/bin/env python3

import rospy

import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt

class camera_sim():
    def __init__(self):
        rospy.init_node('image_to_receiver', anonymous=False)
        self.pubcam1 = rospy.Publisher("/pub_img_under", Image, queue_size=10)
        self.pubcam2 = rospy.Publisher("/pub_img_under2", Image, queue_size=10)

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
        h, l, s = cv2.split(255 - cvt_hls_img)
        
        lower = (0, 0, 0)
        upper = (255, 255, 255)
        filtered_hls_img = cv2.inRange(cvt_hls_img, lower, upper)
        img_result = cv2.bitwise_and(cv_image, cv_image, mask = filtered_hls_img)
        # cv2.imshow('img_result',cvt_hls_img)


        # under
        zero_img = np.zeros_like(cv_image)
        region_under = np.array([[(0, height), (0, height*60/100), (width/2, height*40/100), (width, height*60/100), (width, height)]], dtype = np.int32)
        region_mask_under = cv2.fillPoly(zero_img, region_under, (255, 255, 255))
        masked_under_img = cv2.bitwise_and(img_result, region_mask_under)

        # under  // warp
        src_under = np.float32([[0,height], [width,height], [width*25/100,height*70/100], [width*70/100,height*70/100]])
        dst_under = np.float32([[width*15/100,height], [width*85/100,height], [width*15/100,0], [width*85/100,0]])
        M_under = cv2.getPerspectiveTransform(src_under, dst_under)
        Minv_under = cv2.getPerspectiveTransform(dst_under, src_under)
        warped_under_img = cv2.warpPerspective(masked_under_img, M_under, (width, height))

        self.detect_stopline(warped_under_img, 130)  
        
        # under
        try:
            self.pubcam1.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        except CvBridgeError as e:
            print("publish error")
            print(e)

        try:
            self.pubcam2.publish(self.bridge.cv2_to_imgmsg(self.image_roi))
        except CvBridgeError as e:
            print("publish error")
            print(e)


    def detect_stopline(self,cal_image, low_threshold_value):
        #stopline_roi, _, _ = self.set_roi(cal_image, 250, 350, 10)
        self.image = self.image_processing(cal_image, low_threshold_value)
        self.image_roi = self.image[200:300,300:500]
        self.img_show("Original", self.image_roi)
        print('이미지 전체 픽셀 개수 : {}'.format(self.image_roi.size))
        
        if cv2.countNonZero(self.image_roi) > 12000:
            print("stopline")
        else:
            print("not stopline")

    def image_processing(self, image, low_threshold_value):
        #self.blur = cv2.GaussianBlur(image, (5, 5), 0)
        _, self.L, _  = cv2.split(image)
        _, self.lane = cv2.threshold(self.L, low_threshold_value, 255, cv2.THRESH_BINARY)
        # cv2.imshow("L", self.L)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return self.lane

    def cam_shutdown(self):
        print("I'm dead!")

    def img_show(self,title='image', img=None, figsize=(8 ,5)):
        plt.figure(figsize=figsize)        
        if type(img) == list:
            if type(title) == list:
                titles = title
            else:
                titles = []
    
                for i in range(len(img)):
                    titles.append(title)
    
            for i in range(len(img)):
                if len(img[i].shape) <= 2:
                    rgbImg = cv2.cvtColor(img[i], cv2.COLOR_GRAY2RGB)
                else:
                    rgbImg = cv2.cvtColor(img[i], cv2.COLOR_BGR2RGB)
    
                plt.subplot(1, len(img), i + 1), plt.imshow(rgbImg)
                plt.title(titles[i])
                plt.xticks([]), plt.yticks([])
    
            plt.show()
        else:
            if len(img.shape) < 3:
                rgbImg = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            else:
                rgbImg = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
            plt.imshow(rgbImg)
            plt.title(title)
            plt.xticks([]), plt.yticks([])
            plt.show()
    
    
if __name__ == "__main__":
    rp = camera_sim()