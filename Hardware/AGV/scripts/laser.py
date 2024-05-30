#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
import atexit

class LaserOn:
    def __init__(self):
        rospy.init_node('laser_on', anonymous=True)
        self.pub = rospy.Publisher('laser_on', String, queue_size=10)
        self.run_launch = "roslaunch myagv_odometry myagv_active.launch"
        atexit.register(self.radar_close, self.run_launch)  # 프로그램 종료 시 radar_close 호출

    def radar_open(self):
        def radar_high():
            GPIO.setmode(GPIO.BCM)
            time.sleep(0.1)
            GPIO.setup(20, GPIO.OUT)
            GPIO.output(20, GPIO.HIGH)

        radar_high()
        time.sleep(0.05)
        self.pub.publish(self.run_launch)

    def radar_close(self, run_launch):
        def radar_low():
            GPIO.setmode(GPIO.BCM)
            time.sleep(0.1)
            GPIO.setup(20, GPIO.OUT)
            GPIO.output(20, GPIO.LOW)

        radar_low()
        time.sleep(0.05)
        self.pub.publish("kill " + run_launch)

if __name__ == '__main__':
    laser_on = LaserOn()
    laser_on.radar_open()
    rospy.spin()  # ROS 노드가 종료될 때까지 대기