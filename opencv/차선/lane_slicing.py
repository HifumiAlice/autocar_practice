#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64,Int16,Float32MultiArray,String
import matplotlib.pyplot as plt
from math import *

class IMGParser:
    def __init__(self):
        rospy.init_node('camera', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        

        rospy.spin()

    def callback(self, data):

        np_arr = np.frombuffer(data.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        pub_str = rospy.Publisher('/commands/servo/position',Float64,queue_size = 1)
        pub_speed = rospy.Publisher('/commands/motor/speed',Float64,queue_size = 1)
        
        blue = (255,0,0)
        green = (0,255,0)
        red = (0,0,255)
        yellow = (0,255,255)
        
        # color
        def hsv(img) :
            img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            return img_hsv
        img_hsv = hsv(img_bgr)
        
        def gray(img) : 
            img_Gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY) 
            return img_Gray
        
        #(w,h) = (800,480)
        
        size = [img_bgr.shape[1],img_bgr.shape[0]] 
        
        center_offset = 100 
        boundary_offset = 200
            
        
        #이진화
        def threshold(img) :
            _,img_BINARY_127 = cv2.threshold(img,0,255,cv2.THRESH_BINARY)
            return img_BINARY_127
        
        # 가우시안
        def gaussion(img) : 
            gaussion_blur = cv2.GaussianBlur(img,(5,5),0)
            return gaussion_blur
        
        
        
        # roi
        def roi(image):
            x = int(image.shape[1])
            y = int(image.shape[0])

    
            _shape = np.array(
                [[int(0), int(y)],[int(0), int(360)],[int(x),int(360)],[int(x),int(y)]])

            mask = np.zeros_like(image)

            if len(image.shape) > 2:
                channel_count = image.shape[2]
                ignore_mask_color = (255,) * channel_count
            else:
                ignore_mask_color = 255

            cv2.fillPoly(mask, np.int32([_shape]), ignore_mask_color)
            masked_image = cv2.bitwise_and(image, mask)

            return masked_image
        
        roi_img = roi(img_bgr)
        
        
        
    
        # 버드아이 
        def warpping(roi_img) : 
    
            center_offset = 100 
            boundary_offset = 200
        
            src_point1 = (50,530)
            src_point2 = (480,395)
            src_point3 = (790,395)
            src_point4 = (int(size[0]-100),530)
            src_points = np.float32(np.array([src_point1,src_point2,src_point3,src_point4]))
            
            dst_point1 = (0+200,int(size[1]+400))
            dst_point2 = (0+200,0)
            dst_point3 = (1280-200,0)
            dst_point4 = (int(size[0]-200),int(size[1]+400))
            dst_points = np.float32(np.array([dst_point1,dst_point2,dst_point3,dst_point4]))
            
            
            matrix = cv2.getPerspectiveTransform(src_points,dst_points)
            
            dst = cv2.warpPerspective(roi_img, matrix, (1200,1200))

            cv2.imshow("bird_eye_view",dst)
            
            
            #yellow area
            img_hsv = hsv(roi_img)
            yellow_lower = np.array([15,100,127])
            yellow_upper = np.array([45,255,255])
            yello_mask = cv2.inRange(img_hsv,yellow_lower,yellow_upper)
            yellow_area = cv2.bitwise_and(img_hsv,img_hsv,mask = yello_mask)
            
            
            
            # white area
            
            white_lower = np.array([0,0,200])
            white_upper = np.array([255,127,255])
            white_mask = cv2.inRange(img_hsv,white_lower,white_upper)
            white_area = cv2.bitwise_and(img_hsv,img_hsv,mask = white_mask)
            
            
            yello_white = cv2.bitwise_or(yellow_area,white_area)
            warp_img = cv2.warpPerspective(yello_white,matrix,(1200,1200),flags=cv2.INTER_CUBIC)
            
            return warp_img , src_points , dst_points, src_point1, src_point2, src_point3, src_point4, dst_point1,dst_point2,dst_point3,dst_point4
        
        warp_img , src_points , dst_points, src_point1, src_point2, src_point3, src_point4, dst_point1,dst_point2,dst_point3,dst_point4= warpping(roi_img) 
            # warp_img = cv2.warpPerspective(yello_white,matrix,(1000,1000))
        
        warp_gaussian = gaussion(warp_img)
        img_gray = gray(warp_gaussian)
        img_thres = threshold(img_gray)
        
        minv= cv2.getPerspectiveTransform(dst_points,src_points)
       
        # (픽셀)poitn 따는거 
        def cal_point(img_thres) : 
            bottom_half_y = int(img_thres.shape[0]/2)
            histogram = np.sum(img_thres[bottom_half_y:,:],axis=0)
            midpoint = np.int(histogram.shape[0]/2)
            
            return bottom_half_y , histogram , midpoint
        
        bottom_half_y , histogram , midpoint = cal_point(img_thres)
    
        
        
        # plt.hist(histogram)
        # plt.show()
        
        #window (차선 인식 )
        def window(midpoint, img_thres , histogram) :
            left_base = np.argmax(histogram[:midpoint])
            right_base = np.argmax(histogram[midpoint:]) + midpoint 
            
            leftx_current = left_base
            rightx_current = right_base
            
            img_thres = img_thres*255
            out_img = np.dstack((img_thres,img_thres,img_thres))*255       
            
            
            window_num = 10 
            window_height = int(img_thres.shape[0] / window_num)
            margin = 110
            minplx = 20
            
            lanepixel = img_thres.nonzero()
            lanepixel_y = np.array(lanepixel[0])
            lanepixel_x = np.array(lanepixel[1])
            
            left_lane_idx = []
            right_lane_idx = []
            
            for window in range(window_num):
        
            
                window_y_low = img_thres.shape[0] - (window+1)*window_height
                window_y_high = img_thres.shape[0] - window*window_height
            
                window_x_left_low = leftx_current - margin
                window_x_left_high = leftx_current + margin
                window_x_right_low = rightx_current - margin
                window_x_right_high = rightx_current + margin
                
                cv2.rectangle(out_img,(window_x_left_low,window_y_low),(window_x_left_high,window_y_high),green,2)
                cv2.rectangle(out_img,(window_x_right_low,window_y_low),(window_x_right_high,window_y_high),red,2)
                
                left_idx = ((lanepixel_y>=window_y_low) & (lanepixel_y<window_y_high)  & (lanepixel_x >= window_x_left_low) & (lanepixel_x < window_x_left_high)).nonzero()[0]
                right_idx = ((lanepixel_y>=window_y_low) & (lanepixel_y<window_y_high) & (lanepixel_x >= window_x_right_low) & (lanepixel_x < window_x_right_high)).nonzero()[0]
                
                left_lane_idx.append(left_idx)
                right_lane_idx.append(right_idx) 
                
                # img_thresh = cv2.cvtColor(img_thres,cv2.COLOR_GRAY2BGR)
                
                if len(left_lane_idx) >= minplx : 
                    leftx_current = np.int(np.mean(lanepixel_x(left_idx)))
                if len(right_lane_idx) >= minplx : 
                    rightx_current = np.int(np.mean(lanepixel_x(right_idx)))
                    
            left_lane_idx = np.concatenate(left_lane_idx)
            right_lane_idx = np.concatenate(right_lane_idx)
                
            leftx = lanepixel_x[left_lane_idx]
            lefty = lanepixel_y[left_lane_idx]        
            rightx = lanepixel_x[right_lane_idx]  
            righty = lanepixel_y[right_lane_idx]
            
            leftfit= np.polyfit(lefty,leftx,2)
            rightfit = np.polyfit(righty,rightx,2)
            return leftx,rightx,lefty,righty,leftfit , rightfit, out_img
        
        leftx,rightx,lefty,righty,leftfit ,rightfit, out_img = window(midpoint,img_thres,histogram)
        # 각도 
        def cal_angle(leftx,rightx,lefty,righty,leftfit,rightfit,out_img) : 
            xm_per_pix = 0.001 
            ym_per_pix = 0.001
            y_eval = 0.01 
            
            left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix,2)
            right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix,2)
            
            left_curverad = ((1+(2*left_fit_cr[0]*y_eval*ym_per_pix+left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
            right_curverad = ((1+(2*right_fit_cr[0]*y_eval*ym_per_pix+right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
            
            
            
            
            
            steering_rad = (atan(left_curverad)+atan(right_curverad))/2
            steeirng_degree = steering_rad*180 / pi
            steering = int(-steeirng_degree+90)
            
            
            
            
            ploty = np.linspace(0,out_img.shape[0]-1,3)
            
            left_fitx = leftfit[0]*ploty**2+leftfit[1]*ploty + leftfit[2]
            right_fitx = rightfit[0]*ploty**2+rightfit[1]*ploty + rightfit[2]
            center_fit = (left_fitx + right_fitx) / 2
            left_center_fit = (left_fitx + center_fit) / 2
            right_center_fit = (center_fit + right_fitx) / 2
            
            
            # out_img[lefty,leftx] = (0,0,0)
            # out_img[righty,rightx] = (0,0,0)
            center = np.asarray(tuple(zip(center_fit,ploty)),np.int32)
            left = np.asarray(tuple(zip(left_center_fit,ploty)),np.int32)
            right = np.asarray(tuple(zip(right_center_fit,ploty)),np.int32)
            
            
            
            
            
            
            # cv2.polylines(out_img,[center],False,yellow , thickness = 5)
            cv2.polylines(out_img,[left],False,green , thickness = 5)
            cv2.polylines(out_img,[right],False,red , thickness = 5)
            cv2.polylines(out_img,[center],False,yellow , thickness = 5)
            return center, left , right , steering, left_fitx, right_fitx, center_fit , left_center_fit, right_center_fit,ploty 
        
        center, left , right , steering, left_fitx, right_fitx, center_fit , left_center_fit, right_center_fit,ploty  = cal_angle(leftx,rightx,lefty,righty,leftfit,rightfit,out_img)
            
        
        #draw line 
        
        # 차선 범위 인식 
        def draw_line(center, left , right , steering, left_fitx, right_fitx, center_fit , left_center_fit, right_center_fit,ploty):
            warp_zero = np.zeros_like(img_thres).astype(np.uint8)
            color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

            pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
            pts = np.hstack((pts_left, pts_right))

            mean_x = np.mean((left_fitx, right_fitx), axis=0)
            pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])

            cv2.fillPoly(color_warp, np.int_([pts]), (216, 168, 74))
            cv2.fillPoly(color_warp, np.int_([center]), (0, 0, 255))
            
            
        

            newwarp = cv2.warpPerspective(color_warp, minv, (img_bgr.shape[1], img_bgr.shape[0]))
            result = cv2.addWeighted(img_bgr, 1, newwarp, 0.4, 0)
            
            return result
        
        result = draw_line(center, left , right , steering, left_fitx, right_fitx, center_fit , left_center_fit, right_center_fit,ploty)
                
            

        
        
        
    
        cv2.imshow("checking_lane",out_img)
        cv2.imshow("rst",result)
        
        
        
        print(center)
        print(center[1][0])
        if center[0][0] < 500 :
            steering = -steering
        print("steering",steering)   
        
        

        offset = 25.4799995422
        steer = (steering+offset)/(offset*2)
        print(steer)
        
        pub_str.publish(steer)
        
        if abs(steer) > 15 : 
            speed = 300        
            
            print("speed",speed)
            pub_speed.publish(speed)
            
        elif abs(steer) <= 15 :   
            speed = 1200
            
            print("speed",speed)
            pub_speed.publish(speed)
            
        
        cv2.waitKey(1)
        
        # cv2.imshow('threshold',img_thresh )
        
        
        # def sliding
        
        
        
        cv2.line(img_bgr,src_point1,src_point1,blue,10)
        cv2.line(img_bgr,src_point2,src_point2,green,10)
        cv2.line(img_bgr,src_point3,src_point3,red,10)
        cv2.line(img_bgr,src_point4,src_point4,yellow,10)
        cv2.line(img_bgr,dst_point1,dst_point1,blue,20)
        cv2.line(img_bgr,dst_point2,dst_point2,green,20)
        cv2.line(img_bgr,dst_point3,dst_point3,red,20)
        cv2.line(img_bgr,dst_point4,dst_point4,yellow,20)
    
        
        
        cv2.imshow('original',img_bgr)
    
        cv2.waitKey(1)
        



if __name__ == '__main__':
    try:
        image_parser = IMGParser()
    except rospy.ROSInterruptException:
        pass
