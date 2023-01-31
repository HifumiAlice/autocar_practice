#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan


class lidar_cluster :
    def __init__(self) :
        #rospy.Subscriber('scan', LaserScan, self.callback)  #시뮬에서 받음
        rospy.Subscriber('/lidar2D', LaserScan, self.callback)  #시뮬에서 받음        
       

    def callback(self, msg):
        #print(1)
        cluster=[]  #list to store no. of points there in all the clusters
        i=0
        infinity = float('inf') 
        # for i in range(0,11):
        #     print(f"왼쪽 {i}번째 거리 {msg.ranges[i]}")

        # print("다음")
        #following loop to find out the first point while starting from 0 degree
        for j in range(360):
            # if msg.ranges[j]!=infinity:    
            
            if msg.ranges[j]!=2.0:    
                # rospy.loginfo("range of first detected obs = {}".format(msg.ranges[j]))
                i=j
                break
    
        # initialisinig the first point i.e. x0 and y0 
        theta=i*math.pi/180   ## 시작 위치앵글에서 몇도 돌았는지를 구하는 거 
        x0=x=msg.ranges[i]*math.cos(theta)          ## x좌표 구하기 삼각함수임 빗변 * cos(x)
        y0=y=msg.ranges[i]*math.sin(theta)          ## y좌표 구하기 삼각함수임 빗변 * sin(x)
        i+=1
        points=1
        n=-1
        empty=0
        d = infinity  ### 거리 구하기 초기 값 무한대라고 넣음
    
        # loop to compare distance between subsequents points and find out the distance b/w them         
        # and if distance is less than threshold distance they are of same cluster
        # 루프를 사용하여 후속 점 사이의 거리를 비교하고 거리 b/w를 구합니다
        # 거리가 임계값 거리보다 작으면 동일한 클러스터에 속합니다
        for j in range(i,360):
            # if msg.ranges[j]!=infinity:
            if msg.ranges[j]< 2.0 :             # within 2m
                
                theta=msg.angle_min+j*msg.angle_increment 
                x1=msg.ranges[j]*math.cos(theta)
                y1=msg.ranges[j]*math.sin(theta) 
                d = math.sqrt(pow((x1-x),2)+pow((y1-y),2)) # distance  ## 빗변(거리)구하기 루트(x의차 제곱 + y의차 제곱)
                print(msg.ranges[j])
                print("d={}".format(d))
            
            #rospy.loginfo("d={}".format(d))
            

            if d < 0.25 :                                   #threshold distance taken as 0.25   ### 임계값 거리를 0.25로 계산
                points+=1
            else :
                
                # if distance between the end points of two adjacent clusters is more than width of the turtlebot       
                # then we can call it as an empty space through which the bot can pass width may be taken accordingly,  
                # for waffle_pi, width is 0.31m (approx.)                                                               
                # and for burger, width can be taken as 0.18m (approx.)                                                 
                # 인접한 두 군집의 끝점 사이의 거리가 거북이 로봇의 너비보다 큰 경우
                # 그러면 우리는 그것을 봇이 너비를 통과할 수 있는 빈 공간이라고 부를 수 있다,
                # waffle_pi의 경우 폭은 0.31m(약)입니다
                # 그리고 버거의 경우, 폭은 0.18m(약)로 볼 수 있습니다 
                if d > 0.31 :     
                    empty+=1

                cluster.append(points)
                points=1
                n+=1
            x=x1
            y=y1
        d1=math.sqrt(pow((x-x0),2)+pow((y-y0),2)) #calculating distance b/w the first and last point in a 360 degree rotation since they might be of the same cluster 
        if d1<0.25 :                                #if they are of same cluster then reduce the number of clusters by 1
            cluster[0]=cluster[0]+cluster[n]
            cluster.pop(n)
        elif d1>0.31 :
            empty+=1

    ################################################################
        print ("Total no. of clusters: "+str(len(cluster)))
        for j in range(len(cluster)):
            print("No. of points in cluster "+str(j+1)+" is: "+str(cluster[j]))
        print ("Total no. of empty spaces through which the turtlebot can pass: "+str(empty))

def run() :
    rospy.init_node("PointCloudCluster")
    lidar_cluster()
    rospy.spin()

if __name__ == '__main__' :
    run()