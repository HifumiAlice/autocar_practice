#!/usr/bin/env python
# -*-coding:utf-8-*-

import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class img_converter :
    def __init__(self) :
        self.canny_pub = rospy.Publisher("Canny_img", Image, queue_size=5)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color", Image, self.callback)

    def callback(self, data) :
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("converting error")
            print(e)

        canny_img = cv2.Canny(cv_image, 100, 150)

        try:
            self.canny_pub.publish(self.bridge.cv2_to_imgmsg(canny_img, "mono8"))
        except CvBridgeError as e:
            print("publish error")
            print(e)

def run():
    rospy.init_node('image_converter', anonymous=True)
    ic = img_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Program down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run()
