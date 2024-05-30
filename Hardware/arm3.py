#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float64, Float64MultiArray
from pymycobot import MyCobot

# 로봇암 초기화
mc = MyCobot('/dev/ttyACM0', 115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()

def arm_mov_chk():
    time.sleep(0.6)
    while mc.is_moving():
        time.sleep(0.1)

def move_robot_arm(angle):
    mc.send_angle(4, angle, 20)
    # arm_mov_chk()

def move_robot_arm_coords(coords):
    mc.send_coords(coords, 20, 1)  # 속도 20, 모드 1로 움직임
    arm_mov_chk()

def angle_callback(msg):
    global y_angle
    y_angle = msg.data
    move_robot_arm(y_angle)

def coords_callback(msg):
    global robot_arm_coor
    robot_arm_coor = msg.data
    move_robot_arm_coords(robot_arm_coor)

y_angle = 5
rospy.loginfo("Initializing robot arm position")
mc.send_angles([0, 0, 0, 0, -90, 0], 20)
arm_mov_chk()
mc.send_angles([0,113,-125,y_angle,-90,0], 20)
arm_mov_chk()


def main():
    rospy.init_node('robot_arm_control_node')
    rospy.Subscriber('/arm_camera/angle', Float64, angle_callback)
    rospy.Subscriber('/robot_arm/coords', Float64MultiArray, coords_callback)
    rospy.loginfo("로봇암 제어 노드가 시작되었습니다.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("노드를 종료합니다.")

if __name__ == '__main__':
    main()