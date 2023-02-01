# # 앵글은 정확한데 speed는 소수점 6자리했어도 오차가 있음
# speed = 0.1   # -2.26 ~ 2.26
# speed = speed * 1404.7027199 - 134.223278
# speed = 1062.333479 * speed ######### 개인적으로 이게 괜찮은거 같음

# angle = 17 # -19.5 ~ 19.5
# angle = (angle + 19.5) / 39


# # rviz해서 fixed frame을 보고싶은 센서 frame과 맞춰 주고 그럼에도 화면이 안보이면 add를 눌러서 센서 추가해주면 됨


# ##### lidar #####
# # 0~360도임 0도가 정면에서  회전 방향? 왼쪽으로  34번째 각이면 왼쪽으로 34도임 
# # 내가 원하는 각도를 뽑으려면 lidar메시지에서 
# # left_range = ranges[왼쪽의 원하는 각도 시작 : 왼쪽의 원하는 각도 끝]
# # right_range = ranges[오른쪽의 원하는 각도 시작 : 오른쪽의 원하는 각도 끝w]

# roi_degree_offset = 15 #가운데 기준 offset*2각도
# roi_ranges_left = list(msg.ranges[0:roi_degree_offset])       # 앞에 0도 기준 왼쪽으로 35도
# roi_ranges_left = roi_ranges_left[::-1]    # 왼쪽 끝에서 가운대로
# roi_ranges_right = list(msg.ranges[359-roi_degree_offset:359])   # 앞에 0도 기준 오른쪽으로 35도 
# roi_ranges_right = roi_ranges_right[::-1]  # 가운대에서 오른쪽 끝으로
# roi_ranges_angle = roi_ranges_left + roi_ranges_right # 왼쪽 끝에서 오른쪽 끝으로


# '''
# (0.5,1500) (1,300)    
# 1500-300/0.5-1 = y-300/x-1
# 1200/-0.5 = y-300/x-1
# -2400(x-1) = y - 300
# -2400x +2700 = y
# (0,300) (0.5,1500)
# 1500-300/0.5 = y-300/x
# 1200/0.5 = y-300
# 2400x +300

# (0.5,1000)(1,300) 0.5 a 1,b
# b-a/0.5 = y-b/x-1
# ((b-a)/0.5)*(x-1) = y-b

# y = ((b-a)/0.5 * x-1) + b

# (0,300) (0.5,1000) a b
# b-a/0.5 = y-a/x
# y = (b-a/0.5) * x +a


# '''

# '''
# 제어기 할 때
# 로터리 진입할 때 크럼을 얼마나 할건지 알려줘야함
# '''

# a, b = 3000, 300 # (0.5,a) (1,b)  # 0.5~1
# x = ((b-a)/0.5)
# y = ((b-a)/0.5)*(-1) + b
# print(x, y)

# c, d = 300, 3000 #(0,c)(0.5,d)    # 0~0.5
# x = ((d-c)/0.5)
# y = c
# print(x,y)

a = []
for i in range(10):
    a.append(i)

b = a[0:9]
print(b)