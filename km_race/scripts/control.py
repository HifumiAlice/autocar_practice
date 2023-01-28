#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

# Publish
#   Topic Name --> /high_level/ackerman_cmd_mux/input/nav_0
#   MSG Tyle --> ackermann_msgs/AckermannDriveStamped
#   Steering angle : -0.34 ~ 0.34
#   speed : -10m/s ~ 10m/s   (권장 : -2.5 ~ 2.5)

from ackermann_msgs.msg import AckermannDriveStamped
from km_race.msg import dist, filteredlidar, lidar_measure
from std_msgs.msg import String

class Controller():
    def __init__(self) :
        rospy.loginfo("[Control]publish(/lidar_measure) subscribe(/cvtdist, /roidist, /lidar_clustering)")

        # 주행을 위해서 사용하는 좌우 모멘트의 좌표 변수w
        self.dist_x_left = 0
        self.dist_y_left = 0
        self.dist_x_right = 0
        self.dist_y_right = 0

        # CVT 이미지에서 읽은 좌우 모멘트의 좌표 변수
        self.CVT_x_left = 0
        self.CVT_y_left = 0
        self.CVT_x_right = 0
        self.CVT_y_right = 0

        # ROI 이미지에서 읽은 좌우 모멘트의 좌표 변수
        self.ROI_x_left = 0
        self.ROI_y_left = 0
        self.ROI_x_right = 0
        self.ROI_y_right = 0
        
        # 차선의 폭이 350mm 일때, crop image size가 320 --> 카메라가 유지해야할 좌측과 우측 차선의 거리는 315 (185 + 130)
        # 주행하기 위하여 유지해야하는 차와 차선과의 거리
        # self.ref_dist_right = 185      # crop image size가 320일때
        self.ref_dist_right = 165        # crop image size가 300일때
        self.ref_dist_left = 130         # <==== 차선의 중심에 놓고 거리를 확인하여 변경해야 한다.

        # 카메라 데이터 소스와 기준 차선 지정
        self.data_source = "CVT"        # DEFAULT SOURCE = CVT, (CVT, ROI)
        self.select_LR = "R"            # R, L

        # 필요없는 상수 =========
        # self.dist_x_minlimit = 30
        # self.dist_x_maxlimit = 290

        # CVT와 ROI 이미지가 차선을 인식하는지 아닌지 표시하는 플래그
        self.ROI_process = True         # roilane_callback 
        self.CVT_process = True

        # Driving Mode
        self.driving_mode = 'F'         # F : Forward, C : Lane Change, O : Obs <== steering_angle 값 계산에 사용 (calc_steering_angle)
        self.Lane_Driving = True        # True = Lane Driving, False : Obstacle Driving <== lidar_clustered_Callback 에서만 설정

        # Obstacle Moving Status
        self.obs_Moving = 'H'         # 물체가 움직일 경우 'Y', 안움직일때 'N', 판명되지 않을 때 'H'  - lidar_clustered_Callback :  에서 처리됨

        # Start Obstacle Driving : from 0.7m 
        self.obsDriving = False

        # Obstacle이 존재하는지
        self.Obs_exist = False          # lidar_clustered_Callback 에서 설정

        # LiDAR Measure Mode
        self.Lidar_measure_mode = 1     # setMeasureMode에서 사용, 값을 1 ~ 5까지 변경하면 timer_callbac이 publish하여 다른 범위의 데이터를 스캔함

        # Driving Parameter
        self.speed_limit = 0.5
        self.speed_min = 0.4
        self.speed_normal = 0.4
        self.speed_follow = 0.5

        self.steering_rate = 0.001

        # Steering Angle Limit <== calc_steering_angle() 에서 계산
        self.angle_follow_limit = 0.02
        self.angle_change_lane_limit = 0.08
        self.angle_obs_limit = 0.1

        # Camera Data
        self.crop_image_size = 300

        # LiDAR Data    Topic(filteredlidar)
        self.no_obstacle = 0
        self.sizes_of_obstacle = []
        self.obs_coord = []
        self.obs_theta = []
        self.obs_range = []
        self.smallest_obs = 0
        self.next_obs = 0

        # ChangeLane 관련 필드
        self.first_pos_saving = False   # 첫번째 검출된 물체 1개의 위치를 저장했는지 알리는 플래그
        self.oldSize_of_Obs = 0      # 첫번째 검출된 물체의 위치를 저장 (size가 10cm 이상인 물체)
        self.changeLaneCount = 0

        # 검출된 1개 Obs의 위치 저장 (오른쪽 )

        self.init_param(speed=self.speed_normal)

        self.cvtsubscriber = rospy.Subscriber("cvtdist", dist, self.cvtlane_callback )
        self.roisubscriber = rospy.Subscriber("roidist", dist, self.roilane_callback )
        self.lidarsubscriber = rospy.Subscriber("lidar_clustering", filteredlidar, self.lidar_clustered_Callback )

        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size = 1)
        self.lidar_measure_pub = rospy.Publisher("lidar_measure", lidar_measure, queue_size = 3)

        rospy.Timer(rospy.Duration(1.0/30), self.timer_callback )

    # LiDAR 제어 ====================================================================

    # lidar 의 측정범위 설정을 위한 publisher 
    # mode 반지름	각도	Radian	        현길이
    #  0    3.0	   20	0.34906585	  1.041889066 --> 물체 판별, 움직임에 적합
    #  1    2.0	   40	0.698131701	  1.368080573 --> 물체 판별, 움직임에 적합 (움직일 때, 0.7에 정지 & mode=3으로 변경, 물체가 없으면 mode=2로 변경)
    #  2    2.0	   20	0.34906585	  0.694592711 --> 직진시 적합, 물체가 있으면 mode=2 로 변경
    #  3    0.7	   60	1.047197551	  0.7         --> 움직이는 물체 있는 경우 물체가 없어질 때까지 stop
    #  4    0.7	  120	2.094395102	  1.212435565 --> 라바콘 주행중에 적합 (라바콘 지난 경우 체크)

    def setMeasureMode(self, dist=2.0, angle=40) :
        lidarm_pub = lidar_measure()

        if self.Lidar_measure_mode == 0 :
            lidarm_pub.measure_length = 3.0
            lidarm_pub.measure_degree = 20
        if self.Lidar_measure_mode == 1 :
            lidarm_pub.measure_length = 2
            lidarm_pub.measure_degree = 40
        elif self.Lidar_measure_mode == 2 :
            lidarm_pub.measure_length = 2.0
            lidarm_pub.measure_degree = 20
        elif self.Lidar_measure_mode == 3 :
            lidarm_pub.measure_length = 0.7
            lidarm_pub.measure_degree = 60
        elif self.Lidar_measure_mode == 4 :
            lidarm_pub.measure_length = 0.7
            lidarm_pub.measure_degree = 120
        elif self.Lidar_measure_mode == 5 :         # For obstacleDriving
            lidarm_pub.measure_length = 2
            lidarm_pub.measure_degree = 140
        elif self.Lidar_measure_mode == 6 :
            lidarm_pub.measure_length = dist
            lidarm_pub.measure_degree = angle

        self.lidar_measure_pub.publish(lidarm_pub);

    # LiDAR 데이터를 기준으로 좌표 드라이빙
    #  물체 수에 따라 Driving 이 바뀌어야 함
    #  물체 0개 (Lane_Driving = True, Obs_exist = False)
    #  물체 1개 (Lane_Driving = True, Obs_exist = True)
    #     - 움직이는 물체 (obs_Moving = Y) --> stop 후 출발 (movingObsDriving)
    #     - 고정 물체 (obs_Moving = N)--> lane 변경 (changeLaneDriving)
    #  1m 이내에 물체(라바콘) 2개 이상 (Lane_Driving = False)
    #     - 물체 사이로 이동 (obstacleDriving)

    def lidar_clustered_Callback(self, _data) :
        # rospy.loginfo("[Controller][lidar_clustered]={}".format(_data))
        
        self.getLidarData(_data)                              # Subscribe한 lidar 데이터를 필드에 저장

        self.obs_Moving = 'H'

        if _data.no_obstacle == 0 :
            self.Lane_Driving = True
            self.Obs_exist = False
            self.first_pos_saving = False
            self.oldSize_of_Obs = 0
            self.obs_Moving = 'H'
        elif _data.no_obstacle == 1 :                         # 물체가 없거나 1개 있을 때
            self.Lane_Driving = True
            self.Obs_exist = True
            if self.obs_Moving == 'H' :
                self.obs_Moving = self.check_moving_obs(_data)    # 물체의 움직임 확인

            rospy.loginfo("[Control][lidar_clustered_Callback] obs_Moving({}), oldSize_of_Obs({}), sizes_of_obstacle({})".
                format(self.obs_Moving, self.oldSize_of_Obs, self.sizes_of_obstacle[0]))

        elif _data.no_obstacle > 1 :                # 물체가 2개 이상 검색되었을 때
            if (_data.ranges[0].right < 2.0) :      # 1m 이내일 경우 --> Lane_Driving을 False로 변경
                self.Lane_Driving = False
                self.Obs_exist = True
                self.first_pos_saving = False
                self.oldSize_of_Obs = 0
                self.obstacle_data = _data
                self.obs_Moving = 'H'
            else :
                self.Lane_Driving = True
                self.Obs_exist = True
                self.first_pos_saving = False
                self.oldSize_of_Obs = 0
                self.obs_Moving = 'H'

            rospy.loginfo("[Control][lidar_clustered_Callback] LMM({}), Lane_Driving({}), obs_Moving({}), no_obs({}), range({})".
                format(self.Lidar_measure_mode, self.Lane_Driving, self.obs_Moving, self.no_obstacle, _data.ranges[0].right))


    # Obs의 움직임을 판별
    #  - 물체가 1개일 때만 적용되므로, 물체 1개의 크기를 저장
    #  - 물체의 크기가 변하면(5cm 이상 차이나면) --> 움직이는 것으로 간주 --> True 반환
    #  - 물체까지의 거리가 2m --> 1m 크기가 변화 없으면 --> 고정 --> False 반환
    # 연관된 필드
    #  - first_pos_saving : True, (False)
    #  - oldSize_of_Obs : 처음 발견된 물체의 크기 (0)
    #  - obs_Moving : 'Y', 'N', ('H') (움직임 표시  H - 판명전)
    # 연관된 필드는 changeLaneDriving( ) 끝나고 초기화한다.

    def check_moving_obs(self, _data) :
        # rospy.loginfo("[Control][check_moving_obs=({})".format(_data))
        # 2m부터 측정 시작
        # Obs가 1개인 경우 --> oldSize_of_Obs에 저장
        if self.no_obstacle == 1 :
            if not self.first_pos_saving :
                # rospy.loginfo("[Control][oldSize_of_Obs=({})".format(self.oldSize_of_Obs))

                if self.sizes_of_obstacle[0] > 0.02 :        # 2cm 이상 물체
                    self.oldSize_of_Obs = self.sizes_of_obstacle[0]
                    self.first_pos_saving = True
        
            # 들어오는 데이터와 계속 비교 --> size가 5cm 이상 차이나면 움직이는 것으로 간주
            else :
                if abs(self.sizes_of_obstacle[0] - self.oldSize_of_Obs) > 0.05 : 
                    return 'Y'

        # 1m 까지 측정
        if self.obs_Moving == 'H' :             
            if abs(self.obs_coord[0].rightX) < 1.0 :
                return 'N'
            else :
                return 'H'
        else : 
            return self.obs_Moving


    # Subscribe한 lidar 데이터를 필드에 저장
    # km_race/filteredLidar Msg
    #  int32 no_obstacle            : 물체의 갯수
    #  float32[] sizes_of_obstacle  : 물체의 크기(오른쪽부터)
    #  km_race/coords[] obs_coord   : 물체의 좌표 (오른쪽부터)
    #    float32 rightX             : 물체의 오른쪽 (x, y)
    #    float32 rightY
    #    float32 leftX              : 물체의 왼쪽 (x, y)
    #    float32 leftY
    #  km_race/obs_range[] ranges   : 물체의 각 모서리 거리
    #    float32 left
    #    float32 right
    #  km_race/obs_theta[] theta    : 물체의 각 모서리 각도 (radian)
    #    float32 right
    #    float32 left
    #  int32 smallest_obs           : 검출된 물체 중 가장 가까운 거리에 있는 것의 인덱스 (위의 물체 정보를 사용하기 위해서 필요)
    #  int32 next_obs               : 검출된 물체 중 2번째로 가까운 거리에 있는 것의 인덱스

    def getLidarData(self, _data) :
        if _data is not None :  
            self.no_obstacle = _data.no_obstacle
            self.sizes_of_obstacle = _data.sizes_of_obstacle
            self.obs_coord = _data.obs_coord
            self.obs_range = _data.ranges
            self.obs_theta = _data.theta
            self.smallest_obs = _data.smallest_obs
            self.next_obs = _data.next_obs
        else :
            self.no_obstacle = 0
            self.sizes_of_obstacle = []
            self.obs_coord = []
            self.obs_range = []
            self.obs_theta = []
            self.smallest_obs = 0
            self.next_obs = 0

        # rospy.loginfo("[Control][getLidarData]no_obstacle({}), sizes_of_obstable({}), obs_coord({}), smallest_obs({}), next_obs({})".
        #     format(self.no_obstacle, self.sizes_of_obstacle, self.obs_coord, self.smallest_obs, self.next_obs))


    # Timer Callback =========================================================================
    def timer_callback(self, _event) :
        # rospy.loginfo("[Controller][timer CB]")
        # rospy.loginfo("[Timer CB]Line_Driving({}) {}({})".format(self.Lane_Driving, self.data_source, self.select_LR))
        # rospy.loginfo("[Timer CB]left=({},{}), right=({},{})".format(
        #       self.dist_x_left, self.dist_y_left, self.dist_x_right, self.dist_y_right))
        # rospy.loginfo("[Timer CB]LD({}), CVT({}), ROI({}), DM({}), LMM({}), Obs_exist({}), obs_Moving({})".format(
        #       self.Lane_Driving, self.CVT_process, self.ROI_process, self.driving_mode, self.Lidar_measure_mode, self.Obs_exist, self.obs_Moving))

        self.setMeasureMode()                               # Lidar로 publish

        # Lane Driving / LiDAR Driving 실행
        if self.Lane_Driving :
            # CVT       ROI
            # True   True/False --> CVT
            # False     True    --> ROI
            # False     False   --> CVT / STOP
            if self.CVT_process :                           # CVT Lane이 인식되는 경우
                self.data_source = 'CVT'                    #   데이터 소스를  CVT 로 설정 
            elif self.ROI_process :                         # CVT Lane이 인식 안되고, ROI Lane이 인식되는 경우
                self.data_source = 'ROI'                    #   데이터 소스를  CVT --> ROI 로 변경 
            else :                                          # ROI Lane이 인식 안되고, CVT Lane이 인식되는 경우
                self.data_source = 'CVT'
                self.stop()
                rospy.loginfo("[Timer CB] No Lane detected")

            self.setDistData()

            if self.Obs_exist :                             # 물체가 있는 경우
                if self.obs_Moving == 'Y':                  # 움직이는 물체인 경우
                    self.driving_mode = 'M'
                    self.movingObsDriving()
                elif self.obs_Moving == 'N' :               # 고정 물체인 경우
                    self.driving_mode = 'C'
                    self.changeLaneDriving('L')             # 옆 Lane을 확인하여 인수로 전달하여야 함 <=======================
                else :                                      # 판명되지 않은 경우, 1m까지 주행
                    self.driving_mode = 'F'
                    self.followLaneDriving()
            else :
                self.driving_mode = 'F'
                self.followLaneDriving()

        else :
            self.driving_mode = 'O'
            self.obstacleDriving()

    # CVT subscriber의 callback 함수 ================================================================
    #   - CVT 데이터를 변수에 저장
    #   - 주행을 위한 데이터 설정

    def cvtlane_callback(self, _data) :
        # rospy.loginfo("[control][cvtlane CB]")

        self.CVT_process = True

        self.CVT_x_left = _data.dist_x_left
        self.CVT_y_left = _data.dist_y_left

        self.CVT_x_right = _data.dist_x_right
        self.CVT_y_right = _data.dist_y_right

        if self.driving_mode == 'F' :
            if self.CVT_x_right != 0 :
                self.select_LR = 'R'
            elif (self.CVT_x_left != 0) :
                self.select_LR = 'L'
            else :
                self.CVT_process = False

        # rospy.loginfo("[CVT CB] Source ({}) CVT({}) - left=({},{}), right=({},{})".format(self.data_source, 
        #       self.select_LR, self.dist_x_left, self.dist_y_left, self.dist_x_right, self.dist_y_right))

    # ROI subscriber의 callback 함수 ================================================================
    #   - ROI 데이터를 변수에 저장
    #   - 주행을 위한 데이터 설정

    def roilane_callback(self, _data) :
        # rospy.loginfo("[control][roilane CB]")

        self.ROI_process = True        
        
        self.ROI_x_left = _data.dist_x_left
        self.ROI_y_left = _data.dist_y_left

        self.ROI_x_right = _data.dist_x_right
        self.ROI_y_right = _data.dist_y_right

        if self.driving_mode == 'F' :
            if self.ROI_x_right != 0 :
                self.select_LR = 'R'
            elif (self.ROI_x_left != 0) :
                self.select_LR = 'L'
            else :
                self.ROI_process = False        

        # rospy.loginfo("[ROI CB]Source ({}) ROI({}) - left=({},{}), right=({},{})".format(self.data_source, 
        #     self.select_LR, self.dist_x_left, self.dist_y_left, self.dist_x_right, self.dist_y_right))

    # Driving Control ===================================================================================

    # Lane Driving ----

    def followLaneDriving(self) :
        self.Lidar_measure_mode = 2                         # LiDAR Default scan : distance=2.0, angle=20 
        # self.Lidar_measure_mode = 1                         # LiDAR Default scan : distance=2.0, angle=40 

        self.error_right = self.ref_dist_right - self.dist_x_right    
        self.error_left = self.ref_dist_left - self.dist_x_left           

        # 음수 -> 오른쪽 차선에서 멀다 -> 오른쪽으로 붙여야 한다. -> 바퀴를 오른쪽으로 회전 -> steering_angle을 음수
        # 양수 -> 오른쪽 차선에서 가깝다 -> 왼쪽으로 붙여야 한다. -> 바퀴를 왼쪽으로 회전 -> steering_angle을 양수

        angle = self.calc_steering_angle(self.speed_follow)
        self.driving_pub(angle, self.speed_follow)


    # 물체 사이로 운전 -----
    # 1. LiDAR의 검색 범위를 90도로 설정한다. (Lidar_measure_mode = 5)
    # 2. smallest_obs와 next_obs 중 오른쪽에 있는 obs의 왼쪽 theta와 왼쪽 obs의 오른쪽 theta를 구한다.
    #    - theta 값     오른쪽
    #       양  / 음 -->  양
    #       음대/응소 -->  음대  
    #       양대/양소 -->  양소 
    #      smallest  next     right
    #        -3.1  < -2.5      sm   (smallest값이 작다)
    #         2.5  <  3.1      sm   (smallest값이 작다)
    #        -2.5  > -3.1      ne   (next값이 작다)
    #         3.1  >  2.5      ne   (next값이 작다)
    # 3. 구한 theta 의 중간 지점을 계산 ( 더하여 2로 나눈다 )
    # 4. 중간 theta를 pub 한다.

    def obstacleDriving(self) :
        rospy.loginfo("self.obs_range[0].right({})".format(self.obs_range[0].right))
        self.Lidar_measure_mode = 5            # LiDAR Default scan : distance=2.0, angle=90 --> 0.3m까지 현 0.42m 검색 가능
        
        # search right obs
        right_theta = self.obs_theta[self.smallest_obs]
        left_theta = self.obs_theta[self.next_obs]

        smallest_right = right_theta.right
        next_right = left_theta.right

        rospy.loginfo("[obstableDriving] right_obs({}), right_theta({}), left_obs({}), left_theta({})".format(self.smallest_obs, right_theta, self.next_obs, left_theta))

        # 오른쪽 물체 식별
        # if smallest_right * next_right < 0 :    # 양/음 인경우 --> 양이 오른쪽
        #     if smallest_right > 0 :
        #         self.right_obs = self.smallest_obs
        #         self.left_obs = self.next_obs
        #     else :
        #         self.right_obs = self.next_obs
        #         self.left_obs = self.smallest_obs
        # else :                                      # 양/양, 음/음 인 경우 
        #     if smallest_right > next_right :        # next_right 값이 작은 경우, 더 오른쪽
        #         self.right_obs = self.next_obs
        #         self.left_obs = self.smallest_obs
        #     else :                                  # smallest_right 값이 작은 경우, 더 오른쪽
        #         self.right_obs = self.smallest_obs
        #         self.left_obs = self.next_obs

        self.right_obs = self.smallest_obs
        self.left_obs = self.next_obs
        
        # 두 물체의 중간 theta = (오른쪽 물체의 왼쪽 theta + 왼쪽 물체의 오른쪽 theta ) / 2
        #   - 오른쪽 물체의 left theta : self.obs_theta[self.right_obs][1]
        #   - 왼쪽 물체의 right theta : sekf,obs_theta[self.left_obs][0]
        if self.obs_theta[self.right_obs].left < 0 :
            diff_left = (-np.pi - self.obs_theta[self.right_obs].left )   # -3.1 --> -3.14 - (-3.1) = -3.14 + 3.1 = -0.04
        else :
            diff_left = ( np.pi - self.obs_theta[self.right_obs].left )   # 3.1 --> 3.14 - 3.1 = 0.04

        if self.obs_theta[self.left_obs].right < 0 :
            diff_right = ( -np.pi - self.obs_theta[self.left_obs].right ) * 1.1
        else :
            diff_right = ( np.pi - self.obs_theta[self.left_obs].right ) * 1.1

        middle_theta = -(diff_left + diff_right) / 2

        rospy.loginfo("[obstacleDriving] sm_obs({}), nobs({}), rbos({}), lobs({}), middle_theta({})".format(
            self.smallest_obs, self.next_obs, self.right_obs, self.left_obs, middle_theta))

        self.driving_pub(middle_theta, self.speed_min)

    # 차선 변경 Driving : 정지해 있는 Obstacle이 1개 있는 경우 -----
    #   - _direction : 방향
    def changeLaneDriving(self, _direction) :
        # rospy.loginfo("[control][changeLaneDriving]")

        # 차선이 변경 완료되었는지 확인 : Obs가 없어진 경우
        if self.no_obstacle == 0 :
            self.driving_mode = 'F'
            self.first_pos_saving = False
            self.oldSize_of_Obs = 0
            self.obs_Moving = 'H'
            return


        if _direction == 'L' :              # 모두 양수이어야 왼쪽으로 이동됨
            self.changeLaneCount += 1
            # rospy.loginfo("changeLaneDriving changeLaneCount({})".format(self.changeLaneCount))

            if self.select_LR == 'L' :
                # self.error_left = self.ref_dist_left - self.dist_x_left + self.crop_image_size
                self.error_left = self.crop_image_size
                steering_angle = self.calc_steering_angle(self.speed_min)
            elif self.select_LR == 'R' :
                # self.error_right = self.ref_dist_right - self.dist_x_left - self.crop_image_size  
                self.error_right = self.crop_image_size  
                steering_angle = self.calc_steering_angle(self.speed_min)

            # rospy.loginfo("changeLaneDriving angle=({}), changeLaneCount({})".format(steering_angle, self.changeLaneCount))

            if self.changeLaneCount  < 30 :
                # rospy.loginfo("changeLaneDriving angle=({}), changeLaneCount({})".format(steering_angle, self.changeLaneCount))
                self.driving_pub(steering_angle, self.speed_min)
            elif self.changeLaneCount < 60 :
                # rospy.loginfo("changeLaneDriving angle=({}), changeLaneCount({})".format(0, self.changeLaneCount))
                self.driving_pub(0, self.speed_min)
            elif self.changeLaneCount < 90 :
                # rospy.loginfo("changeLaneDriving angle=({}), changeLaneCount({})".format(-steering_angle, self.changeLaneCount))
                self.driving_pub(-steering_angle, self.speed_min)
            elif self.changeLaneCount > 90:
                self.driving_pub(0, self.speed_follow)
                self.changeLaneCount = 0
                self.driving_mode = 'F'
                # rospy.loginfo("changeLaneDriving({}) angle=({}), changeLaneCount({})".format(self.driving_mode, 0, self.changeLaneCount))


            # if self.changeLaneCount > 5 :
            #     self.driving_pub(-steering_angle, self.speed_min)
            #     self.changeLaneCount = 0
            # else :
            #     self.driving_pub(steering_angle, self.speed_min)

        elif _direction == 'R' :
            if self.select_LR == 'L' :
                # self.error_left = self.ref_dist_right - self.dist_x_right + self.crop_image_size
                self.error_left = -self.crop_image_size
                steering_angle = self.calc_steering_angle(self.speed_min)
            elif self.select_LR == 'R' :
                # self.error_right = self.ref_dist_left - self.dist_x_right -self.crop_image_size  
                self.error_right = -self.crop_image_size  
                steering_angle = self.calc_steering_angle(self.speed_min)            

            # rospy.loginfo("changeLaneDriving angle=({}), changeLaneCount({})".format(steering_angle, self.changeLaneCount))

            if self.changeLaneCount  < 30 :
                rospy.loginfo("changeLaneDriving angle=({}), changeLaneCount({})".format(steering_angle, self.changeLaneCount))
                self.driving_pub(steering_angle, self.speed_min)
            elif self.changeLaneCount < 60 :
                rospy.loginfo("changeLaneDriving angle=({}), changeLaneCount({})".format(0, self.changeLaneCount))
                self.driving_pub(0, self.speed_min)
            elif self.changeLaneCount < 90 :
                rospy.loginfo("changeLaneDriving angle=({}), changeLaneCount({})".format(-steering_angle, self.changeLaneCount))
                self.driving_pub(-steering_angle, self.speed_min)
            elif self.changeLaneCount > 90:
                self.driving_pub(0, self.speed_follow)
                self.changeLaneCount = 0
                self.driving_mode = 'F'
                rospy.loginfo("changeLaneDriving({}) angle=({}), changeLaneCount({})".format(self.driving_mode, 0, self.changeLaneCount))
    
    # 움직이는 Obstacle이 1개 있는 경우 -----
    # 움직임 감지
    # 움직임에 적합 (움직일 때, 0.7에 정지 & mode=3으로 변경, 물체가 없으면 mode=2로 변경)

    def movingObsDriving(self) :
        # Lidar 설정 dist=2.0m, angle=40, 현=1.37m
        # self.Lidar_measure_mode = 1     
        # self.setMeasureMode()               # Lidar로 publish

        # 0.7m 에서 stop
        if self.no_obstacle == 1 :
            if abs(self.obs_coord[0].rightX) < 0.7 or abs(self.obs_coord[0].leftX) < 0.7 :
                self.stop()
        
        # mode=3으로 변경 --> lidar로 publish 는 timer_callback에서 함
        #   - distance=0.7, angle=60, 현의 길이가 0.7 차선내에 있는 물체만 감지

        # self.Lidar_measure_mode = 3         

    # 주행 정지 -----

    def stop(self) :
        self.driving_pub(0.0, 0.0)
        rospy.loginfo("Stop Vehicle!")

    # 주행 데이터 설정 ===================================================================

    # 주행을 위한 모멘트 데이터 설정 
    # 우선 순위 Moment : CVT --> ROI
    # 우선 순인 Lane : R --> L

    def setDistData(self) :
        if (self.data_source == "CVT") :
            self.dist_x_left = self.CVT_x_left
            self.dist_y_left = self.CVT_y_left
            self.dist_x_right = self.CVT_x_right
            self.dist_y_right = self.CVT_y_right

        elif (self.data_source == "ROI") :
            self.dist_x_left = self.ROI_x_left
            self.dist_y_left = self.ROI_y_left
            self.dist_x_right = self.ROI_x_right
            self.dist_y_right = self.ROI_y_right
            

    # 주행을 위한 Publisher  ================================================================

    def driving_pub(self, angle, speed) :
        # rospy.loginfo("[Control][driving_pub]DM({}), ds({}), LR({}), errorR({}), errorL({}), angle({}), speed({})".
        # format(self.driving_mode, self.data_source, self.select_LR, self.error_right, self.error_left, angle, speed))

        publishing_data = AckermannDriveStamped()
        publishing_data.drive.steering_angle = angle

        if (speed > self.speed_limit) :
            publishing_data.drive.speed = self.speed_limit
        else :
            publishing_data.drive.speed = speed

        publishing_data.header.stamp = rospy.Time.now()
        publishing_data.header.frame_id = "base_link"
        self.drive_pub.publish(publishing_data)
    
    # 주행을 위한 필드 (speed, steering_rate) 초기화 ===========================================

    def init_param(self, speed = 0.5, steering_rate = 0.001 ) :
        self.speed = speed
        self.steering_rate = steering_rate

    # Steering Angle 계산 ===================================================================
    
    def calc_steering_angle(self, speed) :
        if self.select_LR == 'L' :
            angle = self.error_left * (self.steering_rate / speed)
        if self.select_LR == 'R' :
            angle = self.error_right * (self.steering_rate / speed)

        # if self.driving_mode == 'F' :
        #     if angle > self.angle_follow_limit :
        #         angle = self.angle_follow_limit 
        #     elif angle < -self.angle_follow_limit :
        #         angle = -self.angle_follow_limit                       
        # elif self.driving_mode == 'C' :
        #     if angle > self.angle_change_lane_limit :
        #         angle = self.angle_change_lane_limit
        #     elif angle < -self.angle_change_lane_limit :
        #         angle = -self.angle_change_lane_limit
        # elif self.driving_mode == 'O' :
        #     if angle > self.angle_obs_limit :
        #         angle = self.angle_obs_limit
        #     elif angle > -self.angle_obs_limit :
        #         angle = -self.angle_obs_limit
        
        return angle
        


def run():
    rospy.init_node("Controller")
    Controller()
    rospy.spin()

if __name__ == '__main__' :
    run()
