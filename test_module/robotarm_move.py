#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float64
from pymycobot import MyCobot

# 로봇암 초기화
mc = MyCobot('/dev/ttyACM0', 115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()

# 초기 위치 설정
y_angle = 5
mc.send_angles([0, 0, 0, 0, -90, 0], 20)
time.sleep(0.6)
mc.send_angles([0, 113, -125, y_angle, -90, 0], 20)
time.sleep(0.6)

# 로봇암이 움직임을 완료했는지 확인하는 함수
def arm_mov_chk():
    time.sleep(0.6)
    while mc.is_moving():
        time.sleep(0.1)

# 로봇암을 움직이는 함수
def move_robot_arm3(angle):
    mc.send_angle(4, angle, 20)
    # arm_mov_chk()

def angle_callback(msg):
    global y_angle
    y_angle = msg.data
    move_robot_arm3(y_angle)

def main():
    rospy.init_node('robot_arm_control_node')
    
    # 서브스크라이버 초기화
    rospy.Subscriber('/arm_camera/angle3', Float64, angle_callback)
    rospy.loginfo("로봇암 제어 노드가 시작되었습니다.")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("노드를 종료합니다.")

if __name__ == '__main__':
    main()
