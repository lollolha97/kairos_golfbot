from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('/dev/ttyACM0', 115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()

mc.send_angles([0,0,0,0,-90,0], 20) #로봇암 영점 시작


time.sleep(0.6)                 #메세지 전달 delay
while mc.is_moving():           # 로봇암이 움직이는 동안 대기
    time.sleep(0.1)             # CPU 사용을 줄이기 위해 작은 대기 시간 추가
mc.set_gripper_state(1,20,1)
time.sleep(0.6)                 #메세지 전달 delay
while mc.is_gripper_moving():   # 그리퍼가 움직이는 동안 대기
    time.sleep(0.1)             # CPU 사용을 줄이기 위해 작은 대기 시간 

mc.set_gripper_value(12, 20)    # 그리퍼 닫기
time.sleep(0.6)                 #메세지 전달 delay
while mc.is_gripper_moving():   # 그리퍼가 움직이는 동안 대기
    time.sleep(0.1)             # CPU 사용을 줄이기 위해 작은 대기 시간 

mc.set_gripper_value(100, 20)   # 그리퍼 열기
time.sleep(0.6)

coords = mc.get_coords()        # 좌표 잘 설정됐는지 확인
print("Current Coordinates of MyCobot:")
print("X: {:.2f}, Y: {:.2f}, Z: {:.2f}, Rx: {:.2f}, Ry: {:.2f}, Rz: {:.2f}".format(*coords))
angles = mc.get_angles()
print("Current Angles of MyCobot:")
print("angles : {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(*angles))
