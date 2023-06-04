#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import GPSMessage
from std_msgs.msg import Float32MultiArray,String,Float64,Int64
from std_msgs.msg import MultiArrayDimension
from pyproj import Proj  ### 없으면 터미널에서 pip3 install pyproj을 입력해 받아주자  그래도 안되면 깃허브 pyproj에 들어가서 clone하고 catkin_make해주자


class erp_gps():
    def __init__(self):
        rospy.init_node('gps', anonymous=True)

    
        #subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.spin()
        
        
    def gpsCB(self,data):
        msg = Float32MultiArray()
        msg_sector = Int64()
        pub_utm = rospy.Publisher('utm',Float32MultiArray,queue_size=1)
        pub_sector = rospy.Publisher('/sector',Int64,queue_size = 3)
        
        
        lon = data.longitude
        lat = data.latitude 
        
        e2u_zone = int(divmod(lon,6)[0])+31
        e2u_conv= Proj(proj='utm', zone=e2u_zone, ellps='WGS84')
        utmx, utmy = e2u_conv(lon, lat)
        if lat<0:
            utmy=utmy+10000000
        

        # print("latitude {}".format(data.latitude))
        # print("longitude {}".format(data.longitude))
        # print("eastOffset {}".format(data.eastOffset))
        # print("northOffset {}".format(data.northOffset))
        
        # print("UTM zone : {}".format(e2u_zone))
        # print("UTM Easting : {}".format(utmx))
        # print("UTM Northing :  {}".format(utmy))
        x = round(utmx - 322944.13,2)
        y = round(utmy - 4164675.78,2)
        utm = [x,y]
        #print (type(utm[0]))
        
        
        ## -4.5
        if 0<= utm[0] <= 1.0 and -5.6<= utm[1] <= -5.2 :
            sector = -1
        elif -7.1 <= utm[0] < 1.6  and -5.6 <= utm[1] <= -5.25 :     #utm[0] >= -8.5 and utm[0] < -4.5 and utm[1]<=-5.25 and utm[1]>=-5.6 : 
            sector = 1  
        elif  3.1 <= utm[0] <= 7.6 and  -5.6 <= utm[1] <= -3.75 :   #utm[0] >= 3.1 and utm[0] <= 5.1 and utm[1]<=-5.25 and utm[1]>=-5.6 : 
            sector = 2  
        elif 6.7 <= utm[0] <= 8 and -3.75 <= utm[1] <= 1.33 :
            sector = 3
        elif 6.7 <= utm[0] < 10 and 1.34 <= utm[1] <= 5 :            #utm[0] >= 6.7 and utm[0] <=8 and utm[1]<=5 and utm[1]>= 2.5 : 
            sector = 4  
        elif 10 <= utm[0] <= 12.5 and 3 <= utm[1] <= 5 :            #utm[0] >= 10 and utm[0] <= 12.50 and utm[1]<=5 and utm[1]>=3: 
            sector = 5  
        elif 11.5 <= utm[0] <= 12.8 and 1.5 <= utm[1] < 3.0 :      #utm[0] >= 11.5 and utm[0] <= 12.8 and utm[1]<= 2.5 and utm[1]>=1.5: 
            sector = 6  
        elif 13.5 <= utm[0] <= 14.2 and -2.0 <= utm[1] <= 0 :             #utm[0] >= 14 and utm[0] <= 15 and utm[1]<=0 and utm[1]>=-1: 
            sector = 7  
        elif 14.6 <= utm[0] <= 19.6 and -1.4 <= utm[1] <= 3:       #utm[0] >= 16.2 and utm[0] <= 17.4 and utm[1]<=0 and utm[1]>=-0.7: 
            sector = 8
        elif -5.8 <= utm[0] <= 17.5 and 4.8 <= utm[1] <= 5.8:
            sector = 9
        
        else:
            sector = 0
        #### 위치 2개 정도만 더 추가하면 좋을거 같다.
        
        ### 토픽 보낼거 저장
        msg.data = [x,y]
        msg_sector.data = sector
        
        ### 프린트로 확인하기
        print("x : {},   y : {}". format(utm[0], utm[1])) 
        print(f"미션 번호 : {sector}")
        
        #### Publish하기
        #pub_utm.publish(msg)
        pub_sector.publish(msg_sector)   
        
        
        # 중심 utm easting = 322944.1349584 utm nothing = 4164675.7849382


    
if __name__ == '__main__':
    try:
        gps=erp_gps()
    except rospy.ROSInterruptException:
        pass