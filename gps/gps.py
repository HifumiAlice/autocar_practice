#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import GPSMessage
from std_msgs.msg import Float32MultiArray,String,Float64
from std_msgs.msg import MultiArrayDimension
from pyproj import Proj 


class erp_gps():
    def __init__(self):
        rospy.init_node('gps', anonymous=True)

    
        #subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.spin()
        
        


    def gpsCB(self,data):
        
        
        
        msg = Float32MultiArray()
        msg_sector = Float64()
        pub_utm = rospy.Publisher('utm',Float32MultiArray,queue_size=1)
        pub_sector = rospy.Publisher('sector',Float64,queue_size=1)
        
    
        
        
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
        print (type(utm[0]))
        
        msg.data = [x,y]
        
        print("x : {}". format(utm[0])) 
        print("y : {}". format(utm[1]))
        
        pub_utm.publish(msg)
        
        if utm[0] >= -8.5 and utm[0] < -4.5 and utm[1]<=-5.25 and utm[1]>=-5.6 : 
            sector = 1 
        if utm[0] >= 3.1 and utm[0] <= 5.1 and utm[1]<=-5.25 and utm[1]>=-5.6 : 
            sector = 2
        if utm[0] >= 6.7 and utm[0] <=8 and utm[1]<=5 and utm[1]>= 2.5 : 
            sector = 3
        if utm[0] >= 10 and utm[0] <= 12.50 and utm[1]<=5 and utm[1]>=3: 
            sector = 4
        if utm[0] >= 11.5 and utm[0] <= 12.8 and utm[1]<= 2.5 and utm[1]>=1.5: 
            sector = 5
        if utm[0] >= 14 and utm[0] <= 15 and utm[1]<=0 and utm[1]>=-1: 
            sector = 6
        if utm[0] >= 16.2 and utm[0] <= 17.4 and utm[1]<=0 and utm[1]>=-0.7: 
            sector = 7
       
        msg_sector.data = sector
        pub_sector.publish(msg_sector)   
        print(sector)
        
       
        
        
        # 중심 utm easting = 322944.1349584 utm nothing = 4164675.7849382


    
if __name__ == '__main__':
    try:
        gps=erp_gps()
    except rospy.ROSInterruptException:
        pass