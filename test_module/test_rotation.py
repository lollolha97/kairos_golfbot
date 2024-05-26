import rospy
import time
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class CVControl:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.detection_subscriber = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.detection_callback, queue_size=1, buff_size=2**24)
        self.bridge = CvBridge()
        self.last_detected_positions = []  # Detected positions of people
        self.step = 0

    def detection_callback(self, data):
        self.process_detections(data)
        if self.is_person_detected() and self.step == 0:
            self.move_toward_target()
        else:
            self.stop_robot()

    def process_detections(self, data):
        self.detected = False
        self.largest_area = 0
        self.largest_area_center = 320  # Image center horizontally
        current_time = time.time()

        for box in data.bounding_boxes:
            if box.Class == "person" and box.probability > 0.2:
                center, area = self.calculate_box_center_and_area(box)
                if area > self.largest_area:
                    self.update_detected_person(center, area, current_time)

    def calculate_box_center_and_area(self, box):
        centerX = (box.xmin + box.xmax) // 2
        area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
        return centerX, area

    def update_detected_person(self, center, area, current_time):
        self.detected = True
        self.largest_area = area
        self.largest_area_center = center
        self.last_detected_positions.append((center, current_time))
        # Remove old positions
        self.last_detected_positions = [pos for pos in self.last_detected_positions if current_time - pos[1] < 5]

    def is_person_detected(self):
        return self.detected

    def move_toward_target(self):
        target_center = 320
        dead_zone_width = 20

        # 중앙점에 대한 각도 조절
        if abs(self.largest_area_center - target_center) <= dead_zone_width:
            angular_velocity = 0
        else:
            angular_velocity = -0.005 * (self.largest_area_center - target_center)

        # 기본값으로 linear Y 이동
        linear_velocity_y = 0.2  # 원하는 기본 Y 축 속도 설정
        self.send_command(linear_velocity_y, angular_velocity)

    def stop_robot(self):
        self.send_command(0, 0)

    def send_command(self, linear_velocity_y, angular_velocity):
        command = Twist()
        command.linear.y = linear_velocity_y
        command.angular.z = angular_velocity
        self.velocity_publisher.publish(command)
        rospy.loginfo(f"cmd_vel published - Linear Velocity Y: {command.linear.y}, Angular Velocity: {command.angular.z}")

def main():
    rospy.init_node('cv_control_tracking', anonymous=True)
    ctrl = CVControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
