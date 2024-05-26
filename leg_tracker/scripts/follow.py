#!/usr/bin/env python

import rospy
from leg_tracker.msg import PersonArray
from geometry_msgs.msg import Twist
import math

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower', anonymous=True)
        
        # Subscriber to the /people_tracked topic
        self.people_sub = rospy.Subscriber('/people_tracked', PersonArray, self.people_callback)
        
        # Publisher to the /cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.linear_speed = 0.45  # Fixed linear speed of the robot
        self.max_angular_speed = 1.0  # Maximum angular speed of the robot
        self.follow_distance = 0.8  # Distance to keep from the target
        self.rate = rospy.Rate(50)  # Loop rate in Hz
        self.target_position = None

        self.follow_person()

    def people_callback(self, msg):
        min_distance = float('inf')
        closest_person = None

        for person in msg.people:
            distance = math.sqrt(person.pose.position.x**2 + person.pose.position.y**2)
            if distance < min_distance:
                min_distance = distance
                closest_person = person.pose.position

        self.target_position = closest_person

    def follow_person(self):
        while not rospy.is_shutdown():
            if self.target_position:
                # Calculate the distance and angle to the target person
                distance = math.sqrt(self.target_position.x**2 + self.target_position.y**2)
                angle_to_target = math.atan2(self.target_position.y, self.target_position.x)
                
                # Create and publish Twist message
                cmd_vel_msg = Twist()
                
                if distance > self.follow_distance:
                    cmd_vel_msg.linear.x = self.linear_speed  # Move forward if the target is too far
                else:
                    cmd_vel_msg.linear.x = 0.0  # Stop moving forward if within follow distance
                
                if abs(angle_to_target) > math.radians(5):
                    cmd_vel_msg.angular.z =  3*self.max_angular_speed * (angle_to_target / math.pi)  # Scale angular speed by angle
                else:
                    cmd_vel_msg.angular.z = 0.0  # No rotation needed if within 30 degrees

                self.cmd_vel_pub.publish(cmd_vel_msg)
            else:
                # Stop the robot if no target person is found
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel_msg)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        PersonFollower()
    except rospy.ROSInterruptException:
        pass
