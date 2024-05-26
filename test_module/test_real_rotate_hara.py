import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class CVControl:
    def __init__(self):
        rospy.init_node('cv_control_compressed', anonymous=True)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.camera_subscriber = rospy.Subscriber("/camera/face_detected", Bool, self.is_forward_callback, queue_size=1)
        self.is_forward = False

    def rotate_target(self):
        while not rospy.is_shutdown():
            if  self.is_facing_forward():
                break
            else: 
                self.send_command(-0.1, 0.1, 0.2)

        self.stop_robot()

    def send_command(self, linearX_velocity, angular_velocity, linearY_velocity=0):
        command = Twist()
        command.linear.x = linearX_velocity * 0.5
        command.angular.z = angular_velocity * 2
        command.linear.y = linearY_velocity
        self.velocity_publisher.publish(command)
        rospy.loginfo(f"cmd_vel published - Linear Velocity: {command.linear.x}, Angular Velocity: {command.angular.z}, Linear Y: {command.linear.y}")

    def stop_robot(self):
        self.send_command(0, 0)

    def is_forward_callback(self, msgs):
        self.is_forward = msgs.data

    def is_facing_forward(self):
        return self.is_forward


def main():
    control = CVControl()
    control.rotate_target()



if __name__ == '__main__':
    main()