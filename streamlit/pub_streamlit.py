#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32

global step
step = 0

global step_signal
step_signal = None
str = "agv_moving"
str1 = "following_mode"
str2 = "adjusting camera perspective and composition"
str3 = "video recoding"
str4 = "analyze_pose_video"
str5 = "stop_signal_agv_stop"

global start_signal_published
start_signal_published = False

def step_callback(msg):
    global step_signal
    step_signal = msg.data

def state():
    global step_signal
    global start_signal_published
    rospy.init_node("step_control", anonymous=True)
    pub = rospy.Publisher('step_str', String, queue_size=10)
    rate = rospy.Rate(1)
    rospy.Subscriber("step_control", Int32, step_callback)

    while not rospy.is_shutdown():
        global step
        if step_signal == 0:
            step = 0
            if not start_signal_published:
                rospy.loginfo('start_signal_agv_move')
                start_signal_published = True
                pub.publish("로봇이 이동을 시작합니다.")
                rate.sleep()
            else:
                rospy.loginfo(str)
            # pub.publish(str)
            rate.sleep()

        elif step_signal == 10:
            step = 10
        elif step_signal == 20:
            step = 20
        elif step_signal == 30:
            step = 30
        elif step_signal == 40:
            step = 40
        elif step_signal == 50:
            step = 50
        elif step_signal == 60:
            step = 60
        elif step_signal == 100:
            step = 100

        if step_signal == 100:
            rospy.loginfo(str5)
            pub.publish("로봇을 정지합니다.")
            rate.sleep()

        if step == 0 or step == 10:
            rospy.loginfo(str)
            pub.publish("이동중입니다...")
            rate.sleep()
        elif step == 20 or step == 30 or step == 40:
            rospy.loginfo(str2)
            pub.publish("녹화 준비중입니다...")
            rate.sleep()
        elif step == 50:
            rospy.loginfo(str3)
            pub.publish("녹화를 시작합니다.")
            rate.sleep()
        elif step == 60:
            rospy.loginfo(str4)
            pub.publish("분석중입니다...")
            rate.sleep()
        else:
            pass

    rospy.spin()

if __name__ == '__main__':
    try:
        state()
    except rospy.ROSInterruptException:
        pass
