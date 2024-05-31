import rospy
from geometry_msgs.msg import Twist
from detection_msgs.msg import BoundingBoxes
import time

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.integral = 0
        self.previous_error = 0
        self.Kp = 0.0015  # 기존 Kp보다 낮춤
        self.Ki = 0   # 기존 Ki보다 낮춤 0.00000005
        self.Kd = 0.00015   # 새로운 미분 계수 추가 0.0005
        self.target_area = 35000
        self.target_area_threshold = 1500
        self.largest_area = 0
        self.target_center = 320  # Assuming the width of the frame is 640 pixels
        self.largest_area_center = 0
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.is_person_detected_flag = False
        self.detection_subscriber = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.detection_callback, queue_size=1, buff_size=2**24)  # buff_size 추가
        self.detected = False
        self.last_detection_time = time.time()
        self.last_detected_positions = []

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

    def is_person_detected(self):
        return self.detected

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Stopping the robot.")

    def evaluate_stopped_condition(self):
        if len(self.last_detected_positions) > 1 and all(abs(pos[0] - self.last_detected_positions[0][0]) <= 30 for pos in self.last_detected_positions):
            if time.time() - self.last_detected_positions[0][1] >= 2:
                print ("Person stopped for 2 seconds")

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

        error = self.target_center - self.largest_area_center
        self.integral += error
        derivative = error - self.previous_error  # 미분 계산

        angular_velocity = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 양 방향으로 돌아갈 수 있도록 절대값이 0.1 이상이 되도록 설정
        if angular_velocity ==0:
            angular_velocity = -0.001

        elif angular_velocity < 0.1 and angular_velocity > -0.1:
            print(f'{angular_velocity}')
            angular_velocity = 0
            

        linear_velocity = -0.000015 * (self.largest_area - self.target_area) * 2
        linear_velocity = max(-0.25, min(0.25, linear_velocity))    

        self.previous_error = error  # 현재 오류 값을 이전 오류 값으로 저장

        # 로봇 제어 명령
        self.send_command(linear_velocity, angular_velocity)

if __name__ == "__main__":
    robot_controller = RobotController()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        robot_controller.move_toward_target()
        rate.sleep()
