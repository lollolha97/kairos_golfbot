#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float64
from pymycobot import MyCobot

# 로봇암 초기화
mc = MyCobot('/dev/ttyACM0', 115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()

# 로봇암이 움직임을 완료했는지 확인하는 함수
def arm_mov_chk():
    time.sleep(0.6)
    while mc.is_moving():
        time.sleep(0.1)

def move_robot_arm3(angle):
    rospy.loginfo(f"Moving robot arm to angle: {angle}")
    mc.send_angle(4, angle, 20)
    # arm_mov_chk()

def angle_callback(msg):
    global y_angle
    y_angle = msg.data
    rospy.loginfo(f"Received angle: {y_angle}")
    move_robot_arm3(y_angle)

# 초기 위치 설정
y_angle = 5
rospy.loginfo("Initializing robot arm position")
mc.send_angles([0, 0, 0, 0, -90, 0], 20)
arm_mov_chk()
mc.send_angles([0, 113, -125, y_angle, -90, 0], 20)
arm_mov_chk()

# 로봇암을 움직이는 함수
def main():
    rospy.init_node('robot_arm_control_node')
    rospy.loginfo("Starting robot arm control node")    
    # 서브스크라이버 초기화
    rospy.Subscriber('/robot_arm/angle', Float64, angle_callback, queue_size=1)
    rospy.loginfo("Subscribed to /robot_arm/angle topic")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node")

if __name__ == '__main__':
    main()