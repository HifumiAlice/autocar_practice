#! /usr/bin/env python3

# -*- coding:utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64


class StopLane():

    def __init__(self) :
        
        rospy.init_node("StopLane_find")
        self.sub = rospy.Subscriber('/image_jpeg/compressed',CompressedImage,callback = self.callback)
        self.pubcam1 = rospy.Publisher("/pub_img_under", Image, queue_size=10)
        self.pubcam2 = rospy.Publisher("/pub_img_under2", Image, queue_size=10)
        self.pub_speed = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
        
        #### 변수 선언
        self.stop_mode = 0 
        self.cv_bridge = CvBridge()
        self.height, self.width = 720, 1280
        self.srcpoint = np.array([[self.width * 10/100, self.height * 95/100],[self.width * 40/100 ,self.height * 60/100],[self.width * 60/100 ,self.height * 60/100],[self.width * 90/100 ,self.height * 95/100]])
        self.dstpoint = np.array([[self.width * 10/100, self.height * 100/100],[self.width * 10/100 ,self.height * 0/100],[self.width * 90/100 ,self.height * 0/100],[self.width * 90/100 ,self.height * 100/100]])


        rospy.on_shutdown(self.CamShutdown)
        rospy.spin()
        pass

    def callback(self,data):

        ### 이미지 변환
        try: #방식1
            cv2_image = self.cv_bridge.compressed_imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("convertion error")
            print(e)
        #방식2
        #np_arr = np.frombuffer(data.data, dtype = np.uint8) 
        #cv2_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #height , width, channel = cv2_image.shape  720,1280,3
        ######## bird eye로 바꾸기
        src_under = np.float32([self.srcpoint[0],self.srcpoint[1],self.srcpoint[2],self.srcpoint[3]]) # 좌하단, 좌상단, 우상단, 우하단  --> 따고 싶은 포인트를 저 순서대로 땃다는 것
        dst_under = np.float32([self.dstpoint[0],self.dstpoint[1],self.dstpoint[2],self.dstpoint[3]]) # 좌하단, 좌상단, 우상단, 우하단       
        under_mask = cv2.getPerspectiveTransform(src_under,dst_under)  #getPerspectiveTransform함수를 쓰려면 포인트?를 float32자료형으로 맞춰야 사용 가능
        wrap_image = cv2.warpPerspective(cv2_image,under_mask,(self.width,self.height)) # 이미지 중에서 포인터로 찝은 것을 원하는 사이즈로 만들어줌// 아마 내가 집은 영역보다 작게는 에러 뜰 듯?
        
        ## 정지선 인식
        L= wrap_image[480:580,300:700] #40000    #[600:720,300:700] 픽셀 : 48000   ### 차가 정지할 때 관성이 있음으로 차량 속도에 따라 로이 범위를 조절해 줘야함 y값만 조절 해주면 될거 같음
        H,L,S = cv2.split(cv2.cvtColor(L,cv2.COLOR_BGR2HLS))
        _,L_roi = cv2.threshold(L,127,255,cv2.THRESH_BINARY)   ### 흰색 차선 찾기 좋음 즉 정지선 구분하기 좋다 --> L 채널이 흰색을 찾기가 좋다는 뜻이다.
        
        if cv2.countNonZero(L_roi) > 28000:   ## 화면 내에 일정이상 흰색점이 발견됐다면 정지선이다.  전체 화면 픽셀수 대비 70% 이상이면 정지선으로 인식해도 될듯?   
            #self.stop_mode = 1
            #self.pub_speed.publish(0)
            print("stopline")
        else:
            print("not stopline")

        print(cv2.countNonZero(L_roi)) # countnonzero 사용을 위해서 채널 한개인 이미지를 써야함 
        # 픽셀값 구하는 코드 지워도 됨
        print('이미지 전체 픽셀 개수 : {}'.format(L_roi.size)) # 픽셀값을 모르겠다면 이걸로 알아볼 수 있다.
   
        ### 이미지 보기
        cv2.imshow("cv2_image",cv2_image)
        cv2.imshow("wrap_image",wrap_image)
        cv2.imshow("self.image_roi",L_roi)
        cv2.waitKey(1)  

        ## publish
        #under
        try:
            self.pubcam1.publish(self.cv_bridge.cv2_to_imgmsg(L_roi)) 
            self.pubcam2.publish(self.cv_bridge.cv2_to_imgmsg(wrap_image,"rgb8"))             
        except CvBridgeError as e:
            print("publish error")
            print(e)
        
    def CamShutdown(self):
        print("Cam is dead!")

    

if __name__ == "__main__":
    SL = StopLane()