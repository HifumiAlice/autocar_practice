

####### bitwise_and 연습 #######
# import cv2
# import numpy as np


# win_name = "color_detect_hsv"



# def nothing (x):
#     pass

# def create_trackbar_init():

#     cv2.createTrackbar('LH',win_name,0,179,nothing)
#     cv2.createTrackbar('LS',win_name,0,255,nothing)
#     cv2.createTrackbar('LV',win_name,0,255,nothing)
#     cv2.createTrackbar('UH',win_name,179,179,nothing)
#     cv2.createTrackbar('US',win_name,255,255,nothing)
#     cv2.createTrackbar('UV',win_name,255,255,nothing)

# def hsv_track(frame):
    
#     # trackbar의 조절값을 변수에 저장
#     Lower_H_Value = cv2.getTrackbarPos("LH",win_name)
#     Lower_S_Value = cv2.getTrackbarPos("LS",win_name)
#     Lower_V_Value = cv2.getTrackbarPos("LV",win_name)
#     Upper_H_Value = cv2.getTrackbarPos("UH",win_name)
#     Upper_S_Value = cv2.getTrackbarPos("US",win_name)
#     Upper_V_Value = cv2.getTrackbarPos("UV",win_name)

#     # hsv영역으로 색영역 전환
#     cvt_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#     # 임계값 boundary 정의
#     lower = np.array([Lower_H_Value,Lower_S_Value,Lower_V_Value])
#     upper = np.array([Upper_H_Value,Upper_S_Value,Upper_V_Value])
#     mask = cv2.inRange(cvt_hsv,lower,upper)

#     result1 = cv2.bitwise_and(frame,frame,mask = mask) #BGR 영역 겹친 부분 확인
#     result2 = cv2.bitwise_and(cvt_hsv,cvt_hsv,mask = mask) #HSV 영역 겹친 부분 확인
#     
#     return lower,upper,cvt_hsv,result1,result2

# if __name__ == "__main__":
    
#     #영상 불러오기 
#     origin_color = cv2.imread('/home/autosun/Downloads/picture/hsv.png')

#     cv2.namedWindow("color_detect_hsv")

#     ######### trackbar ##########
#     create_trackbar_init()
    
#     while True :
#         lower, upper, cvt_hsv,result1, result2 = hsv_track(origin_color)        
#         cv2.imshow("origin",origin_color)
#         cv2.imshow("converted_hsv",cvt_hsv)    
#         cv2.imshow('result1',result1)
#         cv2.imshow('result2',result2)
#         if cv2.waitKey(0) & 0xFF ==27:
#             break

#     cv2.destroyAllWindows()


########## ??? ###########

import cv2
import numpy as np

