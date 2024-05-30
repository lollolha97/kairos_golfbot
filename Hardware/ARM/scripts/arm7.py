#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float64, Float64MultiArray, Int32, Bool
from pymycobot import MyCobot

# 로봇암 초기화
mc = MyCobot('/dev/ttyACM0', 115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()

previous_step = 100
is_robotarm_shrink = False

# 글로벌 변수로 선언
is_arm_moving_pub = None

def arm_mov_chk():
    global is_arm_moving_pub
    time.sleep(0.6)
    while mc.is_moving():
        if is_arm_moving_pub:
            is_arm_moving_pub.publish(True)
        else:
            rospy.logwarn("is_arm_moving_pub is not initialized")
        time.sleep(0.1)
    if is_arm_moving_pub:
        is_arm_moving_pub.publish(False)
    else:
        rospy.logwarn("is_arm_moving_pub is not initialized")

def move_robot_arm(angle):
    mc.send_angles([0, 113, -125, angle, -90, 0], 20)
    arm_mov_chk()

def move_robot_arm_coords(coords):
    mc.send_coords(coords, 60, 1)  # 속도 60, 모드 1로 움직임
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
    global previous_step
    
    step = msg.data

    if previous_step != step:
        previous_step = step

        if step == 0:
            is_robotarm_shrink = True
            mc.send_angles([0, 113, -125, -5, -90, 0], 40)
            arm_mov_chk()

        if step == 10:
            is_robotarm_shrink = True
            coords = [-11.90, -94.60, 235.30, -139.87, 0.83, -121.41]
            mc.send_coords(coords, 60, 1)  # 속도 60, 모드 1로 움직임
            arm_mov_chk()

        if step == 30:
            mc.send_angles([0, 113, -125, -15, -90, 0], 40)
            arm_mov_chk()

        if step == 40:
            mc.send_angles([0, 113, -125, -5, -90, 0], 40)
            arm_mov_chk()

        if step == 60:

            for _ in range(2):
                coords = [-165.30, 92.40, 397.00, -90.00, 2.28, 96.67]
                mc.send_coords(coords, 60, 1)  # 속도 60, 모드 1로 움직임
                arm_mov_chk()

                coords = [-152.70, 0.10, 522.50, -90.53, 3.91, 83.03]
                mc.send_coords(coords, 60, 1)  # 속도 60, 모드 1로 움직임
                arm_mov_chk()

                coords = [-153.00, -142.40, 387.70, -89.91, -2.54, 88.15]
                mc.send_coords(coords, 60, 1)  # 속도 60, 모드 1로 움직임
                arm_mov_chk()

                coords = [-152.70, 0.10, 522.50, -90.53, 3.91, 83.03]
                mc.send_coords(coords, 60, 1)  # 속도 60, 모드 1로 움직임
                arm_mov_chk()

        if step == 100:
            mc.send_angles([0, 113, -125, -5, -90, 0], 40)
            arm_mov_chk()

y_angle = 5

def main():
    global is_arm_moving_pub
    rospy.init_node('robot_arm_control_node')
    is_arm_moving_pub = rospy.Publisher('/is_arm_moving', Bool, queue_size=1)
    rospy.Subscriber('/step_control', Int32, step_callback)
    rospy.Subscriber('/arm_camera/angle', Float64, angle_callback)
    rospy.Subscriber('/arm_camera/coords', Float64MultiArray, coords_callback)
    
    rospy.loginfo("Initializing robot arm position")
    
    rospy.loginfo("로봇암 제어 노드가 시작되었습니다.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("노드를 종료합니다.")

if __name__ == '__main__':
    main()
