#import cv2

#messi = cv2.imread('/home/autosun/Downloads/picture/messi.jpg',cv2.IMREAD_COLOR)
# messi = cv2.imread('/home/autosun/Downloads/picture/messi.jpg',1)
# cv2.namedWindow("img",cv2.WINDOW_AUTOSIZE)
# cv2.imshow('img',messi)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

####### 2 ########

# messi = cv2.imread('/home/autosun/Downloads/picture/messi.jpg',0)
# cv2.imshow('img',messi)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# cv2.imwrite("messi.png",messi)

###### 3 ########

# messi = cv2.imread('/home/autosun/Downloads/picture/messi.jpg',0)
# cv2.imshow('img',messi)

# while True:
#     k = cv2.waitKey(0) 
#     if k == 27:
#         cv2.destroyAllWindows()
#         print("Key : ",k)
#         break
#     elif k == ord('s'):
#         cv2.imwrite('messi_gray.jpg',messi)
#         print("Key : ", k)
#         break
#     else:
#         print("Key : ", k)
        
####### 6 #######

# import cv2
# import numpy as np

# height = 1000
# width = 1500

# # 빈영상 생성
# color = np.zeros((height,width,3),dtype = np.uint8)
# gray = np.zeros((height,width),dtype = np.uint8)

# color[600:700,800:900] = [0,255,0]
# color[300:500,500:1000] = [255,0,0]

# gray[600:700,800:900] = 255
# gray[00:500,500:1000] = 125

# cv2.imshow('color',color)
# cv2.imshow("gray",gray)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

##### 7 도형 그리기 ######

# import cv2
# import numpy as np

# img = np.zeros((512,512,3),dtype = np.uint8)

# img = cv2.rectangle(img,(384,00),(510,128),(0,255,0),3)
# img = cv2.circle(img,(447,63),63,(0,0,255),-1)

# cv2.imshow("res",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

######## 8 마우스 이벤트 ###########

# import cv2
# import numpy as np

# drawing = False
# mode = True
# ix,iy = -1,-1
# # mouse callback fucntion
# def draw_circle(event,x,y,flags,param):
#     # if event == cv2.EVENT_LBUTTONDBLCLK:
#     #     cv2.circle(img,(x,y),100,(255,0,0),-1)

#     global ix,iy,drawing,mode

#     if event == cv2.EVENT_LBUTTONDOWN:
#         if drawing == True:
#             drawing = False
#         else:
#             drawing = True
#         ix,iy = x,y

#     elif event == cv2.EVENT_MOUSEMOVE:
#         if drawing == True:
#             if mode == True:
#                 cv2.rectangle(img,(ix,iy),(x,y),(0,255,0),-1)
#             else:
#                 cv2.circle(img,(x,y),15,(0,0,255),-1)
    


# img = np.zeros((512,512,3),dtype=np.uint8)
# cv2.namedWindow("image")
# cv2.setMouseCallback('image',draw_circle)

# while True:
#     cv2.imshow('image',img)
    
#     # if cv2.waitKey(20) & 0xFF == 27:
#     #     break

#     k = cv2.waitKey(1) & 0xFF
#     if k == ord('m'):
#         mode = not mode
#     elif k == 27:
#         break
# cv2.destroyAllWindows()


########### 9 Track Bar ###########

# import cv2
# import numpy as np

# def nothing(x):
#     pass

# # Create a black image, a window
# img = np.zeros((512,512,3),dtype=np.uint8)
# cv2.namedWindow('image')

# # Create trackbars for color change
# cv2.createTrackbar('R','image',0,255,nothing)
# cv2.createTrackbar('G','image',0,255,nothing)
# cv2.createTrackbar('B','image',0,255,nothing)

# # create switch for ON/OFF functionality
# switch = '0 : OFF \n1 : ON'
# cv2.createTrackbar(switch,"image",0,1,nothing)

# while True:

#     cv2.imshow('image',img)
#     k = cv2.waitKey(1) & 0xFF
#     if k == 27:
#         break

#     # get current positions of four trackbar
#     r = cv2.getTrackbarPos("R",'image')
#     g = cv2.getTrackbarPos("G",'image')
#     b = cv2.getTrackbarPos("B",'image')
#     s = cv2.getTrackbarPos(switch,'image')

#     if s == 0:
#         img[:] = 0
#     else:
#         img[:] = [b,g,r]

# cv2.destroyAllWindows()

######### 10 이미지 처리 기초 ##########

# import cv2
# import numpy as np
# from matplotlib import pyplot as plt

# BLUE = [255,0,0]
 
# img1 = cv2.imread('/home/autosun/Downloads/picture/opencv_logo.png',cv2.IMREAD_COLOR)

# replicate = cv2.copyMakeBorder(img1,30,30,30,30,cv2.BORDER_REPLICATE)
# reflect = cv2.copyMakeBorder(img1,30,30,30,30,cv2.BORDER_REFLECT)
# reflect101 = cv2.copyMakeBorder(img1,30,30,30,30,cv2.BORDER_REFLECT_101)
# wrap = cv2.copyMakeBorder(img1,30,30,30,30,cv2.BORDER_WRAP)
# constant = cv2.copyMakeBorder(img1,30,30,30,30,cv2.BORDER_CONSTANT,value=BLUE)

# plt.subplot(231),plt.imshow(img1,'gray'),plt.title("ORIGINAL")
# plt.subplot(232),plt.imshow(replicate,'gray'),plt.title("REPLICATE")
# plt.subplot(233),plt.imshow(reflect,'gray'),plt.title("REFLECT")
# plt.subplot(234),plt.imshow(reflect101,'gray'),plt.title("REFLECT101")
# plt.subplot(235),plt.imshow(wrap,'gray'),plt.title("WRAP")
# plt.subplot(236),plt.imshow(constant,'gray'),plt.title("CONSTANT")

# plt.show()

###### 11 이미지 연산1 #########

# import cv2
# import numpy as np

# img1 = cv2.imread("/home/autosun/Downloads/picture/flower1.jpg")
# img2 = cv2.imread("/home/autosun/Downloads/picture/flower3.jpg")

# def nothing(x):
#     pass

# cv2.namedWindow("image",cv2.WINDOW_NORMAL)
# cv2.createTrackbar("W","image",0,100,nothing)

# while True:
#     w = cv2.getTrackbarPos("W",'image')

#     dst = cv2.addWeighted(img1,float(100-w)*0.01, img2,float(w)*0.01,0)

#     cv2.imshow('image',dst)

#     if cv2.waitKey(1) & 0xFF == 27:
#         break

# cv2.destroyAllWindows()

###### 11 이미지 연산2 #########

import cv2
import numpy as np

# Load two images
img1 = cv2.imread("/home/autosun/Downloads/picture/messi.jpg")
img2 = cv2.imread("/home/autosun/Downloads/picture/opencv_logo.png")

# I want to put logo on top-left corner , So I create a ROI
rows,cols,channels = img2.shape
#roi = np.zeros((img1[0],img1[1],3),dtype=np.uint8)
roi = img1[0:rows,0:cols]

# Now create a mask of logo and create its inverse mask also
img2gray =  cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
ret, mask = cv2.threshold(img2gray,10,255,cv2.THRESH_BINARY)
mask_inv = cv2.bitwise_not(mask)

# Now black-out the area of logo in ROI
img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)

# Take only regin of logo from logo image
img2_fg = cv2.bitwise_and(img2,img2,mask = mask)

# Put logo in ROI and modify the main image
dst = cv2.add(img1_bg,img2_fg)
img1[0:rows, 0 :cols] =dst

cv2.imshow('res',img1)
cv2.waitKey(0)
cv2.destroyAllWindows()



























