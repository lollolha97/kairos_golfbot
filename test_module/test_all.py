import rospy
import time
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import cv2
import mediapipe as mp
import math
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, model_complexity=1)

bridge = CvBridge()

def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def is_facing_forward(landmarks, width, height):
    left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
    right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
    left_eye = landmarks[mp_pose.PoseLandmark.LEFT_EYE.value]
    right_eye = landmarks[mp_pose.PoseLandmark.RIGHT_EYE.value]

    ls = (int(left_shoulder.x * width), int(left_shoulder.y * height))
    rs = (int(right_shoulder.x * width), int(right_shoulder.y * height))
    le = (int(left_eye.x * width), int(left_eye.y * height))
    re = (int(right_eye.x * width), int(right_eye.y * height))

    eye_line = abs(le[1] - re[1])
    shoulder_line = abs(ls[1] - rs[1])

    eye_distance = calculate_distance(le, re)
    shoulder_distance = calculate_distance(ls, rs)

    face_size = eye_distance
    y_threshold = face_size * 0.2
    distance_ratio_threshold = 0.7

    if eye_line < y_threshold and shoulder_line < y_threshold and shoulder_distance / eye_distance > distance_ratio_threshold:
        return True
    return False

def all_landmarks_visible(landmarks, visibility_threshold=0.5):
    for landmark in landmarks:
        if landmark.visibility < visibility_threshold:
            return False
    return True

def draw_landmarks(image, landmarks, connections, visibility_threshold=0.5):
    for landmark in landmarks:
        if landmark.visibility >= visibility_threshold:
            x = int(landmark.x * image.shape[1])
            y = int(landmark.y * image.shape[0])
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1)

    if connections:
        for connection in connections:
            start_idx = connection[0]
            end_idx = connection[1]
            if landmarks[start_idx].visibility >= visibility_threshold and landmarks[end_idx].visibility >= visibility_threshold:
                start_point = (int(landmarks[start_idx].x * image.shape[1]), int(landmarks[start_idx].y * image.shape[0]))
                end_point = (int(landmarks[end_idx].x * image.shape[1]), int(landmarks[end_idx].y * image.shape[0]))
                cv2.line(image, start_point, end_point, (0, 255, 0), 2)

class CVControl:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.detection_subscriber = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.detection_callback, queue_size=1, buff_size=2**24)
        self.camera_subscriber = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.camera_callback, queue_size=1)
        self.bridge = CvBridge()
        self.last_detected_positions = []  # Detected positions of people
        self.step = 0
        self.is_facing_forward = False

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

    def camera_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image)

            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.pose_landmarks:
                draw_landmarks(image, results.pose_landmarks.landmark, mp_pose.POSE_CONNECTIONS)

                if is_facing_forward(results.pose_landmarks.landmark, image.shape[1], image.shape[0]):
                    self.is_facing_forward = True
                    cv2.putText(image, "Facing Forward", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    
                    if all_landmarks_visible(results.pose_landmarks.landmark):
                        cv2.putText(image, "All Landmarks Visible", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    else:
                        cv2.putText(image, "Not All Landmarks Visible", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                else:
                    self.is_facing_forward = False
                    cv2.putText(image, "Not Facing Forward", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow('MediaPipe Pose', image)
            cv2.waitKey(1)

            if self.step == 30 and self.is_facing_forward:
                self.step_to_next()

        except CvBridgeError as e:
            rospy.logerr("CvBridge 에러: {0}".format(e))

    def step_to_next(self):
        print("Stopping.")
        self.step = 40
        self.step_publisher.publish(40)

def main():
    rospy.init_node('cv_control_tracking', anonymous=True)
    ctrl = CVControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
