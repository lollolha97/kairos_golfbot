import rospy
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self):
        self.detection_subscriber = rospy.Subscriber("/yolov5/detections", BoundingBoxes, 
                                                    self.detection_callback, queue_size=1, buff_size=2**24)
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.largest_area_center = 0
        self.largest_area = 0
        self.person_detected = False

    def detection_callback(self, data):
        self.process_detections(data)

    def process_detections(self, data):
        largest_area = 0
        largest_area_center = 0
        for box in data.bounding_boxes:
            if box.Class == "bottle":
                area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
                if area > largest_area:
                    print("area: ", area)
                    largest_area = area
                    largest_area_center = (box.xmax + box.xmin) / 2
        self.largest_area = largest_area
        self.largest_area_center = largest_area_center
        self.person_detected = largest_area > 0

    def is_person_detected(self):
        return self.person_detected

    def stop_robot(self):
        self.send_command(0, 0, 0, 0)

    def send_command(self, linear_x, linear_y, linear_z, angular_velocity):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)

    def move_toward_target(self):
        target_center = 320
        target_area = 20000
        # target_area = 50000
        dead_zone_width = 20
        if not self.is_person_detected():
            self.stop_robot()
            return

        if abs(self.largest_area_center - target_center) <= dead_zone_width:
            angular_velocity = 0
        else:
            angular_velocity = 0.002 * (target_center - self.largest_area_center)

        linear_velocity_x = -0.0000045 * (self.largest_area - target_area)
        linear_velocity_x = max(-0.25, min(0.25, linear_velocity_x))

        # Calculating linear y and z velocities for circular motion
        linear_velocity_y = 0.1  # Adjust as needed for desired circular path
        linear_velocity_z = 0.1  # Adjust as needed for desired circular path

        self.send_command(linear_velocity_x, linear_velocity_y, linear_velocity_z, angular_velocity)

if __name__ == "__main__":
    rospy.init_node("robot_controller")
    controller = RobotController()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        controller.move_toward_target()
        rate.sleep()
