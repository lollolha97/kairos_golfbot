#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float64, Float64MultiArray, Int32
from pymycobot import MyCobot

# 로봇암 초기화
mc = MyCobot('/dev/ttyACM0', 115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()

is_robotarm_shrink = False

def arm_mov_chk():
    time.sleep(0.6)
    while mc.is_moving():
        time.sleep(0.1)

def move_robot_arm(angle):
    mc.send_angles([0,113,-125,angle,-90,0], 20)
    # arm_mov_chk()

def move_robot_arm_coords(coords):
    mc.send_coords(coords, 60, 1)  # 속도 20, 모드 1로 움직임
    arm_mov_chk()

def angle_callback(msg):
    global y_angle
    y_angle = msg.data
    move_robot_arm(y_angle)

def coords_callback(msg):
    global robot_arm_coor
    robot_arm_coor = msg.data
    move_robot_arm_coords(robot_arm_coor)

def step_callback(msg):
    global is_robotarm_shrink
    step = msg.data
    if step == 10 and is_robotarm_shrink == False:
        is_robotarm_shrink = True
        coords = [-11.90,-94.60,235.30,-139.87,0.83,-121.41]
        mc.send_coords(coords, 60, 1)  # 속도 20, 모드 1로 움직임
        arm_mov_chk()
        is_robotarm_shrink = False
    if step == 100:
        mc.send_angles([0,113,-125,-5,-90,0], 40)
        arm_mov_chk()

y_angle = 5
rospy.loginfo("Initializing robot arm position")
mc.send_angles([0, 0, 0, 0, -90, 0], 20)
arm_mov_chk()
mc.send_angles([0,113,-125,y_angle,-90,0], 20)
arm_mov_chk()

def main():
    rospy.init_node('robot_arm_control_node')
    rospy.Subscriber('/step_control', Int32, step_callback)
    rospy.Subscriber('/arm_camera/angle', Float64, angle_callback)
    rospy.Subscriber('/arm_camera/coords', Float64MultiArray, coords_callback)
    rospy.loginfo("로봇암 제어 노드가 시작되었습니다.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("노드를 종료합니다.")

if __name__ == '__main__':
    main()
