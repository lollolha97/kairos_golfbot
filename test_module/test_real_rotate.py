import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import numpy as np
import math

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, model_complexity=1)

class CVControl:
    def __init__(self):
        rospy.init_node('cv_control_compressed', anonymous=True)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.camera_subscriber = rospy.Subscriber("/arm_camera/image/compressed", CompressedImage, self.camera_callback, queue_size=1)
        self.bridge = CvBridge()
        self.largest_area_center = 320  # 이미지 중심
        self.is_person_detected = False
        self.all_landmarks_visible = False
        self.is_facing_forward = False
        self.target_center = 320

    def rotate_target(self):
        if not self.is_person_detected:
            self.stop_robot()
            return

        if self.is_facing_forward:
            self.stop_robot()
            return

        if not self.all_landmarks_visible:
            angular_velocity = 0.001 * (self.target_center - self.largest_area_center)
            linear_velocity_y = 0.2  # 원형 경로를 위한 속도 조정
            self.send_command(-0.1, angular_velocity, linear_velocity_y)
            return

        angular_velocity = 0.001 * (self.target_center - self.largest_area_center)
        linear_velocity_y = 0.2  # 원형 경로를 위한 속도 조정
        self.send_command(0, angular_velocity, linear_velocity_y)

    def send_command(self, linearX_velocity, angular_velocity, linearY_velocity=0):
        command = Twist()
        command.linear.x = linearX_velocity * 0.5
        command.angular.z = angular_velocity * 2
        command.linear.y = linearY_velocity
        self.velocity_publisher.publish(command)
        rospy.loginfo(f"cmd_vel published - Linear Velocity: {command.linear.x}, Angular Velocity: {command.angular.z}, Linear Y: {command.linear.y}")

    def stop_robot(self):
        self.send_command(0, 0)

    def camera_callback(self, data):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image_rgb)

            if results.pose_landmarks:
                self.is_person_detected = True
                self.all_landmarks_visible = all([lm.visibility > 0.5 for lm in results.pose_landmarks.landmark])
                self.is_facing_forward = is_facing_forward(results.pose_landmarks.landmark, image.shape[1], image.shape[0])
                self.largest_area_center = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].x * image.shape[1])

        except CvBridgeError as e:
            rospy.logerr("CvBridge error: {0}".format(e))

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

def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def main():
    ctrl = CVControl()
    rate = rospy.Rate(10)  # 10Hz 루프 속도 설정

    while not rospy.is_shutdown():
        if ctrl.is_person_detected:
            ctrl.rotate_target()
        rate.sleep()

if __name__ == '__main__':
    main()