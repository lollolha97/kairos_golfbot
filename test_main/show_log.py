#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, String

class StepConverter:
    def __init__(self):
        self.current_step = None
        self.pub = rospy.Publisher('/step_show', String, queue_size=10)
        rospy.Subscriber('/step_control', Int32, self.callback)
        rospy.Timer(rospy.Duration(1), self.timer_callback)

    def callback(self, data):
        self.current_step = data.data
        rospy.loginfo("Received step: %d", self.current_step)

    def timer_callback(self, event):
        if self.current_step is not None:
            step_string = "Current step: " + str(self.current_step)
            rospy.loginfo("Publishing: %s", step_string)
            self.pub.publish(step_string)

def main():
    rospy.init_node('step_control_to_step_show', anonymous=True)
    step_converter = StepConverter()
    rospy.spin()

if __name__ == '__main__':
    main()
