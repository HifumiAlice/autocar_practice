#!/usr/bin/env python3

# -*- coding : utf-8 -*-

import rospy
import time
from ros_basics.msg import CamData,CamControl
from std_msgs.msg import Float64,Int64
from ros_basics.msg import LidarData, LidarControl

class Control():

    def __init__(self):

        #### 변수 설정
        rospy.init_node("Control_Node")

        ########### 구독자 발행자 선언        

        # 시뮬레이션 변수
        self.PubSpeed = rospy.Publisher("/commands/motor/speed",Float64,queue_size=3)
        self.PubAngle = rospy.Publisher("/commands/servo/position",Float64,queue_size=3)

        # 카메라 변수
        self.CamSub = rospy.Subscriber("/CamData",CamData,self.CamCallback)
        self.CamPub = rospy.Publisher("/CamControl",CamControl,queue_size=3)
        self.CamData = CamData()
        self.CamControl = CamControl()

        # gps 변수
        self.GpsSub = rospy.Subscriber("/sector",Int64,self.gpscallback)
        self.gpsmsg = 0

        # 라이다 변수
        self.LidarSub = rospy.Subscriber("/LidarData",LidarData,self.LidarCallback)
        self.LidarPub = rospy.Publisher("/LidarControl",LidarControl,queue_size=3)
        self.LidarData = LidarData()
        self.LidarControl = LidarControl()
        self.rotate_flag = False 

        # 센서의 각도의 따라 속도 저장
        self.CamSpeed = 0
        self.Rotspeed = 0
        self.LabSpeed = 0
        # 제어기 짜다가 급하게 변수 추가
        self.go = False
        self.PreTime = None
        self.NextTime = None
        self.TimeFind = False
        self.LineLoca = True # true : 2차로 False : 1차로
        self.StopFind = False
        self.MoveFind = False 
        self.obsFind = False
        self.RotStop = False

        rospy.Timer(rospy.Duration(1.0/10),self.timer)
        rospy.spin()
        pass

    
    def CamCallback(self,data):
        self.CamData = data 

        pass

    def gpscallback(self,data):
        self.gpsmsg = data.data
        #print(self.gpsmsg)
    
    def LidarCallback(self,data):        
        self.LidarData = data

        pass

    
    def timer(self,_event):  
        
        self.anglefind()
        self.OnOff()        
        angle,speed = self.Find()  
        self.publish(angle,speed)

    def anglefind(self):

        ### 카메라 각도, 속도 처리
        if  0<= self.CamData.CamAngle <= 0.5:
            self.CamSpeed = 1400 * self.CamData.CamAngle + 300 # 1200   

        elif 0.5 < self.CamData.CamAngle <=1:
            self.CamSpeed = -1400 * self.CamData.CamAngle + 1700   

        else:
            self.CamSpeed = 300

        ## 라바콘 각도 및 속도 처리
        if self.LidarData.LabacornAngle != -1.0:  ## 라바콘 주행시에 각도가 나오면 
            if  0<= self.LidarData.LabacornAngle <= 0.5:
                self.LabSpeed = 1200 * self.LidarData.LabacornAngle + 100 # 700 ~ 100
                
            elif 0.5 < self.LidarData.LabacornAngle <=1:
                self.LabSpeed = -1200 * self.LidarData.LabacornAngle + 1300 # 700 ~ 100
                
            else:
                self.LabSpeed = 300
        
        ## 로터리 각도 및 속도 처리
        if self.LidarData.RotaryAngle != -1.0:  # 로터리 주행에서 각도가 나오면
            # print("Distance : ",self.LidarData.RotaryDis)
            # print("로터리 각도 : ",self.LidarData.RotaryAngle)
            if  0<= self.LidarData.RotaryAngle <= 0.5:
                #speed = 600 * angle + 700 # 1000~700                
                if self.LidarData.RotaryDis >= 1.3: #1.3에 700~500이 젤 좋은거 같음 --> 거리의 범위를 바꾸면 각도 구하는 공식을 좀 손봐줘야함 + 속도도 다시 계산
                    if self.rotate_flag : ### 진입할때 왼쪽부터 인식이 되서 문제가 발생 --> 오른쪽이 인식이 된 후에 주행시 왼쪽을 인식하므로 오른쪽 인식되면 참
                        self.Rotspeed = 400 * self.LidarData.RotaryAngle + 300 #500 ~ 300
                    else:
                        self.Rotspeed = 0
                else:
                    self.Rotspeed = 0
            elif 0.5 < self.LidarData.RotaryAngle <=1:
                #speed = -600 * angle + 1300
                if self.LidarData.RotaryDis >= 0.5:
                    self.Rotspeed = -400 * self.LidarData.RotaryAngle + 700 # 500~300 
                    self.rotate_flag = True
                else:
                    self.Rotspeed = 0
            else:
                self.Rotspeed = 0 
                # self.LidarData.RotaryAngle = -1
                # self.rotate_flag = False
        # print("rotate Flag : ",self.rotate_flag)
        

    def OnOff(self):

        ### 카메라 ON/OFF
        self.CamControl.line_flag = True
        self.CamControl.light_flag = False
        self.CamControl.stopline_flag = False

        ### 라이다 ON/OFF
        if self.LidarData.GroupLen >=2 :  # 2개 이상이 그룹핑됐으면 라바콘이됨
            self.LidarControl.roi_deg = 60
            self.LidarControl.Labacorn_flag = True
            self.LidarControl.Rotary_flag = False
            self.LidarControl.obs_flag = False
        else:                             # 1개면 정적/동적 또는 회전교차로  
            self.LidarControl.roi_deg = 20
            self.LidarControl.Labacorn_flag = False
            self.LidarControl.Rotary_flag = False
            self.LidarControl.obs_flag = True

        if self.gpsmsg == 0:  ### 아무데나          
            self.CamControl.line_find = "a"

        elif self.gpsmsg == 1: ## 공터에서 나올 때
            self.CamControl.line_find = "r1"

        elif self.gpsmsg == 2: ## 신호등 가는 길 좌회전           
            self.CamControl.line_find = "l"

        elif self.gpsmsg == 3: ## 신호등 찾는 곳           
            self.CamControl.line_find = "a"
            self.CamControl.light_flag = True
            self.CamControl.stopline_flag = True

        elif self.gpsmsg == 4: ## 신호등 지나고 우회전           
            self.CamControl.line_find = "r2" 

        elif self.gpsmsg == 5: ## 회전 로터리 전 우회전           
            self.CamControl.line_find = "r2"

        elif self.gpsmsg == 6: ## 회전 로터리 대기           
            self.CamControl.line_find = "a"
            self.CamControl.stopline_flag = True
            self.LidarControl.Rotary_flag = True ### 주석 같이 풀거 A
            self.LidarControl.obs_flag = False
            self.LidarControl.roi_deg = 60
            # if self.RotStop == True:
            #     self.CamControl.stopline_flag = False
            
        elif self.gpsmsg == 7: ## 회전 로터리 중 빠져나갈 때           
            self.CamControl.line_find = "r1"
            self.rotate_flag = False

        elif self.gpsmsg == 8: ## 로터리 나온 후 좌회전            
            self.CamControl.line_find = "l"

        elif self.gpsmsg == 9:            
            self.CamControl.line_find = "r1"
        
        else:
            print("아무고토 아닌데?")  

        if self.rotate_flag : # 참이다
            self.LidarControl.Labacorn_flag = False
            self.LidarControl.Rotary_flag = True
            self.LidarControl.obs_flag = False
            self.LidarControl.roi_deg = 60

        pass
    
    def Find(self):
 
        
        # 정지선, 신호등 함수가 켜짐
        if (self.CamControl.light_flag ) and (self.CamControl.stopline_flag) : ## 신호등에서            
            if self.CamData.stopline_find == "s" : # 정지선이고                
                if self.CamData.light_find != "g": # 초록불이 아니라면
                    speed = 0
                    angle = 0.5
                else:                              # 초록불이라면
                    angle = 0.5
                    speed = 1000
                    self.go = True
            else:                                  # 정지선이 아니라면
                speed = 500
                angle = self.CamData.CamAngle
            
            if self.go: #초록불이 한번 인식 됐으면 --> sector4까지 진행
                angle = 0.5
                speed = 1000
                
        # 정지선 신호등 꺼짐 --> 정지
        else:
            if self.LidarData.LabacornAngle != -1: ## 라바콘 주행임
                angle = self.LidarData.LabacornAngle
                speed = self.LabSpeed
                self.LineLoca = True
            elif self.LidarData.RotaryAngle != -1 :  ### 로터리 주행임
                
                if self.CamControl.stopline_flag :
                    if not self.RotStop:
                        if self.CamData.stopline_find =="s":
                            speed = 0
                            angle = 0.5
                            self.RotStop = True
                        
                        else:
                            speed = 500
                            angle = self.CamData.CamAngle
                    else:
                        angle = self.LidarData.RotaryAngle
                        speed = self.Rotspeed  
                else:
                    angle = self.LidarData.RotaryAngle
                    speed = self.Rotspeed

            else:                
                if (self.LidarData.GroupLen == 1) and (self.LidarData.RotaryAngle == -1) : ### 동적 정적 판단 
                    speed = 0
                    angle = 0.5
                    if not self.obsFind:
                        print("동적정적 기다림")
                        print(self.obsFind)
                        if self.LidarData.ObStatus == "s": ## 정적이다.
                            print("정적이다")
                            self.StopFind = True
                            self.obsFind = True
                        
                        elif self.LidarData.ObStatus == "m":
                            print("동적이다")
                            self.MoveFind = True
                            self.obsFind = True

                    else:
                        speed = 0
                        angle = 0.5
                        print("들어오긴하나?")
                        if self.StopFind :
                            if self.LidarData.GroupLen > 0:
                                if self.LineLoca: #True : 2차로임
                                    angle = 0.0
                                    speed = 700
                                else: #False : 1차로임
                                    angle = 1.0
                                    speed = 700
            
                        elif self.MoveFind:
                            if self.LidarData.GroupLen > 0 : # 동적이고 앞에 장애물이 있다.
                                speed = 0
                                angle = self.CamData.CamAngle
                                angle = self.CamData.CamAngle
                                speed = self.CamSpeed
                        else:
                        #     self.StopFind = False
                        #     self.MoveFind = False
                            self.obsFind = False

                elif self.CamControl.stopline_flag: 
                    if self.CamData.stopline_find == "s": #정지선이고                        
                        ### 주석 같이 풀거 A
                        if self.LidarData.RotaryAngle == -1: # 교차로 장애물이 없다면
                            speed = 0
                            angle = 0.5
                            self.rotate_flag = False
                        else:
                            angle = self.LidarData.RotaryAngle
                            speed = self.Rotspeed 
                               
                    else:                                 #정지선이 아니라면
                        speed = 500
                        angle = self.CamData.CamAngle
                
                else:# self.LidarData.GroupLen == 0 :
                    if self.StopFind:
                        self.StopFind = False
                        self.obsFind = False
                        self.LineLoca = not self.LineLoca
                        self.LidarControl.Obstatus = "o"
                    elif self.MoveFind:
                        self.MoveFind = False
                        self.obsFind = False
                        self.LidarControl.Obstatus = "o"
                    else:
                        self.LidarControl.Obstatus = ""

                    angle = self.CamData.CamAngle
                    speed = self.CamSpeed

        print(f"각도 : {angle}, 속도 : {speed}")
        # print(f"로터리 거리 : {self.LidarData.RotaryDis}, 로터리 각도 : {self.LidarData.RotaryAngle}")
        # print("로터리 flag : ",self.rotate_flag)
        return angle,speed
        

    def publish(self, angle, speed):
        
        ### 카메라, 로터리, 라바콘 각도 및 속도를 선택
        
        
        # print(self.gpsmsg)   # ok --> 잘 들어옴
        # print(self.CamData.stopline_find)  # ok --> 잘 들어옴
        print(self.LidarData)  # ok --> 잘 들어옴

        #### publish
        # 카메라에게 publish
        # print(self.CamControl)
        self.CamPub.publish(self.CamControl)

        # 라이다에게 publish
        # print(self.LidarControl.Obstatus)
        self.LidarPub.publish(self.LidarControl)

        # 시뮬레이터에 publish
        self.PubAngle.publish(angle)
        self.PubSpeed.publish(speed)
        if self.gpsmsg == -1:
            if self.go == True:
                self.StopFunc()
        pass
    
    def StopFunc(self):
        angle = 0.5
        speed = 0
        self.PubAngle.publish(angle)
        self.PubSpeed.publish(speed)
    

if __name__ == "__main__":
    control = Control()
