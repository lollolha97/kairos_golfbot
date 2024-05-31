import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
from detection_msgs.msg import BoundingBoxes
import time

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.integral = 0
        self.previous_error = 0
        self.Kp = 0.0015
        self.Ki = 0
        self.Kd = 0.00015
        self.target_area = 35000
        self.target_area_threshold = 3000
        self.largest_area = 0
        self.target_center = 320  # Assuming the width of the frame is 640 pixels
        self.largest_area_center = 0
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.is_person_detected_flag = False
        self.detection_subscriber = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.detection_callback, queue_size=1, buff_size=2**24)
        self.detected = False
        self.last_detection_time = time.time()
        self.last_detected_positions = []
        self.step_publisher = rospy.Publisher("/step_control", Int32, queue_size=1)
        self.step_subscriber = rospy.Subscriber("/step_control", Int32, self.step_callback)
        self.step = 0
        self.use_leg_tracker = False
        self.pub_detected = rospy.Publisher("/detected", Bool, queue_size=1)

    def step_callback(self, data):
        self.step = data.data
        rospy.loginfo(f"Step updated to: {self.step}")
                
    def detection_callback(self, data):
        self.process_detections(data)

        if self.detected:
            self.last_detection_time = time.time()  # 마지막 검출 시간 업데이트
            self.use_leg_tracker = False

        elif time.time() - self.last_detection_time > 3.0:
            self.use_leg_tracker = True

    def process_detections(self, data):
        self.detected = False
        self.largest_area = 0
        current_time = time.time()

        for box in data.bounding_boxes:
            if box.Class == "person":
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
        self.last_detected_positions = [pos for pos in self.last_detected_positions if current_time - pos[1] < 3]
        rospy.loginfo(f"Updated detected positions: {self.last_detected_positions}")

    def is_person_detected(self):
        return self.detected
    
    def update_step(self, step):
        self.step = step
        self.step_publisher.publish(step)
        rospy.loginfo(f"Step updated to: {step}")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Stopping the robot.")

    def evaluate_stopped_condition(self):
        if len(self.last_detected_positions) > 1 and all(abs(pos[0] - self.last_detected_positions[0][0]) <= 30 for pos in self.last_detected_positions):
            if time.time() - self.last_detected_positions[0][1] >= 2:
                rospy.loginfo("Person stopped for 2 seconds")
                self.stop_robot()  # 로봇을 멈추고
                rospy.sleep(1)  # 1초간 대기
                self.update_step(30)
            else:
                rospy.loginfo("Person detected but not stopped for 2 seconds yet")
        else:
            rospy.loginfo("Person not detected in the same position")

    def send_command(self, linear_velocity, angular_velocity):
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Sending command - Linear velocity: {linear_velocity}, Angular velocity: {angular_velocity}")

    def move_toward_target(self):
        if not self.is_person_detected():
            self.stop_robot()
            return

        if abs(self.largest_area - self.target_area) <= self.target_area_threshold:
            self.evaluate_stopped_condition()
            return

        error = self.target_center - self.largest_area_center
        self.integral += error
        derivative = error - self.previous_error  # 미분 계산

        angular_velocity = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 양 방향으로 돌아갈 수 있도록 절대값이 0.1 이상이 되도록 설정
        if angular_velocity == 0:
            angular_velocity = -0.001
        elif -0.1 < angular_velocity < 0.1:
            angular_velocity = 0

        linear_velocity = -0.000015 * (self.largest_area - self.target_area) * 2
        linear_velocity = max(-0.25, min(0.25, linear_velocity))

        self.previous_error = error  # 현재 오류 값을 이전 오류 값으로 저장

        # 로봇 제어 명령
        self.send_command(linear_velocity, angular_velocity)

    def follow_person(self):
        if self.step == 10:
            self.move_toward_target()
            # detected 를 publish
            self.pub_detected.publish(True)
if __name__ == "__main__":
    robot_controller = RobotController()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        robot_controller.follow_person()
        rate.sleep()
