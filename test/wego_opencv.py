#! /usr/bin/env python3
#-*- coding: utf-8 -*-

###### bitwise_and 연습 #######
import cv2
import numpy as np
# 
# 
win_name = "color_detect_hsv"
# 
# 
# 
def nothing (x):
    pass
# 
def create_trackbar_init():
# 
    cv2.createTrackbar('LH',win_name,0,179,nothing)
    cv2.createTrackbar('LS',win_name,0,255,nothing)
    cv2.createTrackbar('LV',win_name,0,255,nothing)
    cv2.createTrackbar('UH',win_name,179,179,nothing)
    cv2.createTrackbar('US',win_name,255,255,nothing)
    cv2.createTrackbar('UV',win_name,255,255,nothing)
# 
def hsv_track(frame):
    # 
    # trackbar의 조절값을 변수에 저장
    Lower_H_Value = cv2.getTrackbarPos("LH",win_name)
    Lower_S_Value = cv2.getTrackbarPos("LS",win_name)
    Lower_V_Value = cv2.getTrackbarPos("LV",win_name)
    Upper_H_Value = cv2.getTrackbarPos("UH",win_name)
    Upper_S_Value = cv2.getTrackbarPos("US",win_name)
    Upper_V_Value = cv2.getTrackbarPos("UV",win_name)
# 
    # hsv영역으로 색영역 전환
    cvt_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# 
    # 임계값 boundary 정의
    lower = np.array([Lower_H_Value,Lower_S_Value,Lower_V_Value])
    upper = np.array([Upper_H_Value,Upper_S_Value,Upper_V_Value])
    mask = cv2.inRange(cvt_hsv,lower,upper)
# 
    result1 = cv2.bitwise_and(frame,frame,mask = mask) #BGR 영역 겹친 부분 확인
    result2 = cv2.bitwise_and(cvt_hsv,cvt_hsv,mask = mask) #HSV 영역 겹친 부분 확인
    
    return lower,upper,cvt_hsv,result1,result2
# 
if __name__ == "__main__":
    # 
    #영상 불러오기 
    origin_color = cv2.imread('/home/autosun/Downloads/picture/hsv.png')
# 
    cv2.namedWindow("color_detect_hsv")
# 
    ######### trackbar ##########
    create_trackbar_init()
    # 
    while True :
        lower, upper, cvt_hsv,result1, result2 = hsv_track(origin_color)        
        cv2.imshow("origin",origin_color)
        cv2.imshow("converted_hsv",cvt_hsv)    
        cv2.imshow('result1',result1)
        cv2.imshow('result2',result2)
        if cv2.waitKey(0) & 0xFF ==27:
            break
# 
    cv2.destroyAllWindows()


########## image thrersholding ###########

# thresholding 임계값 정하는 것

############ bitwise operation ############

############ Geometric Transformation ############

# import cv2
# import numpy as np


# img = cv2.imread('/home/autosun/Downloads/picture/opencv_logo.png')

##### 확대 축소 #####
# 행: height 열 : width
# height, width = img.shape[:2]

# # 이미지 축소
# shrink = cv2.resize(img,None,fx=0.5,fy = 0.5, interpolation = cv2.INTER_AREA)

# # manual size 지정
# zoom1 = cv2.resize(img,(width*2, height*2),interpolation=cv2.INTER_CUBIC)

# # 배수 size 지정
# zoom2 = cv2.resize(img,None, fx=2,fy=2,interpolation=cv2.INTER_CUBIC)

# while True:
#     cv2.imshow('Original',img)
#     cv2.imshow('shrink',shrink)
#     cv2.imshow('zoom1',zoom1)
#     cv2.imshow('zoom2',zoom2)
    
#     if cv2.waitKey(0) & 0xff == 27:
#         break

# cv2.destroyAllWindows()

####### translation #######

# rows, cols = img.shape[0:2]

# # 변환 행렬, x축으로 10, y축으로 20 이동
# M = np.float32([[1,0,30],[0,1,60]])

# dst = cv2.warpAffine(img,M,(cols,rows))
# cv2.imshow('original',img)
# cv2.imshow('translation',dst)

# cv2.waitKey(0)
# cv2.destroyAllWindows()

############ image Smoothing ############

# import cv2
# import numpy as np

# ###### image blurring ######
# img = cv2.imread("/home/autosun/Downloads/picture/mauntain.jpg")
# img = cv2.resize(img,None,fx = 3,fy = 3,interpolation=cv2.INTER_LINEAR)
# cv2.namedWindow("image")
# cv2.imshow('image',img)
# cv2.waitKey(0)

# def nothing(x):
#     pass

# cv2.createTrackbar('k','image',0,50,nothing)

# while True:
#     k = cv2.getTrackbarPos('k','image')

#     if k == 0:
#         k = 1
    
#     kernel = np.ones((k,k),np.float32)/(k**2)  # 커널은 행렬이다.
#     dst = cv2.filter2D(img,-1,kernel)
#     cv2.imshow('image',dst)
#     if cv2.waitKey(1) & 0xFF == 27:
#         break
# cv2.destroyAllWindows()

############ Morphological Transformation ############

############ image Gradient ############

############ image pyramid ############

############ image contours ############

############ image histogram ############

############ Hough Transformation ############

######## hough lines ########

# import cv2
# import numpy as np

# def nothing(x):
#     pass


# img = cv2.imread("/home/autosun/Downloads/picture/chessboard2.jpg",cv2.IMREAD_COLOR)
# img_original = img.copy()
# img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# cv2.namedWindow("image")
# cv2.createTrackbar('min',"image",0,255,nothing)
# cv2.createTrackbar('max',"image",255,512,nothing)

# while True:

#     min_value = cv2.getTrackbarPos("min","image")
#     max_value = cv2.getTrackbarPos("max","image")

#     img_Edges = cv2.Canny(img_gray,min_value,max_value)    
#     lines = cv2.HoughLines(img_Edges,1,np.pi/180,90) #이 방법도 있고 단 모든 점에서 찾는거라 좀 오래 걸릴수가 있음 houghlinesp도 있다
    
    
#     for i in range(len(lines)):
#         for rho,theta in lines[i]:
#             a = np.cos(theta)
#             b = np.sin(theta)
#             x0 = a*rho
#             y0 = b*rho
#             x1 = int(x0 + 1000*(-b))
#             y1 = int(y0 + 1000*(a))
#             x2 = int(x0 - 1000*(-b))
#             y2 = int(y0 - 1000*(a))
    
#             cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

#     res = np.vstack((img_original,img))
#     cv2.imshow("image",res)
#     if cv2.waitKey(1) & 0xFF == 27:
#         break

# cv2.destroyAllWindows()

###### hough linesp #######
# import cv2
# import numpy as np

# def nothing(x):
#     pass


# img = cv2.imread("/home/autosun/Downloads/picture/chessboard2.jpg",cv2.IMREAD_COLOR)
# img_original = img.copy()
# img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# cv2.namedWindow("image")
# cv2.createTrackbar('threshold',"image",50,255,nothing)
# cv2.createTrackbar('minLineLength',"image",50,150,nothing)
# cv2.createTrackbar('maxLineGap',"image",5,30,nothing)
# cv2.createTrackbar('min',"image",50,255,nothing)
# cv2.createTrackbar("max","image",150,512,nothing)

# while True:
#     img = img_original.copy()
#     min_value = cv2.getTrackbarPos("min","image")
#     max_value = cv2.getTrackbarPos("max","image")
#     threshold = cv2.getTrackbarPos("threshold","image")
#     minLineLength = cv2.getTrackbarPos("minLineLength","image")
#     maxLineGap = cv2.getTrackbarPos("maxLineGap","image")

#     img_Edges = cv2.Canny(img_gray,min_value,max_value)
#     lines = cv2.HoughLinesP(img_Edges,1,np.pi/360,threshold,minLineLength,maxLineGap)    
    
#     for i in range(len(lines)):        
#         for x1,y1,x2,y2 in lines[i] :
#             cv2.line(img,(x1,y1),(x2,y2),(0,0,255),3)

#     #res = np.vstack((img_original,img))
#     cv2.imshow('canny',img_Edges)
#     cv2.imshow("image",img)
#     if cv2.waitKey(1) & 0xFF == 27:
#         break

# cv2.destroyAllWindows()



########## 
