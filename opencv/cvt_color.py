#! usr/bin/env python3
# -*- coding:utf-8 -*-

######## 컬러 스페이스 변경 ##########
# import cv2
# import numpy as np
# img = cv2.imread('/home/autosun/Downloads/picture/cherryblossom.jpg')
# hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

# lower_blue = np.array([0, 0, 0])
# upper_blue = np.array([255,255,255])

# mask = cv2.inRange(hsv, lower_blue, upper_blue)

# result = cv2.bitwise_and(img, img , mask = mask)

# cv2.namedWindow('BGR',cv2.WINDOW_NORMAL)
# cv2.namedWindow('HSV',cv2.WINDOW_NORMAL)
# cv2.namedWindow('RESULT',cv2.WINDOW_NORMAL)

# cv2.imshow('HSV',hsv)
# cv2.imshow('BGR',img)
# cv2.imshow('RESULT',result)

# cv2.waitKey(0)
# cv2.destroyAllWindows()


##########
import cv2
import numpy as np

img = cv2.imread('/home/autosun/Downloads/picture/hsl_top.jpg')
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

cv2.namedWindow("BGR",cv2.WINDOW_NORMAL)
cv2.namedWindow("HSV",cv2.WINDOW_NORMAL)
cv2.namedWindow("RESULT",cv2.WINDOW_NORMAL)
deg = 255/360
lower_Color = np.array([deg*0,0,0])
upper_Color = np.array([deg*360,255,255])

# hsv 영역에서 원하는 부분만 뽑아낸 거임
mask = cv2.inRange(hsv, lower_Color, upper_Color)
result = cv2.bitwise_and(img, img , mask = mask) #원하는 부분만 뽑아내서 겹칠 이미지에 원하는 부분만 찾아내는 거임

cv2.imshow("BGR",img)
cv2.imshow("HSV",hsv)
cv2.imshow("RESULT",result)

cv2.waitKey(0)
cv2.destroyAllWindows()





