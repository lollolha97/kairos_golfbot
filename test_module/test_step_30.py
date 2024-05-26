import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from detection_msgs.msg import BoundingBoxes
import cv2
import numpy as np
import mediapipe as mp
import math

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, model_complexity=1)  # Mediapipe 초기화

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
    y_threshold = face_size * 0.1
    distance_ratio_threshold = 0.5

    if eye_line < y_threshold and shoulder_line < y_threshold and shoulder_distance / eye_distance > distance_ratio_threshold:
        return True
    return False

def feet_landmarks_visible(landmarks, visibility_threshold=0.5):
    left_foot_index = landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value]
    right_foot_index = landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value]

    if left_foot_index.visibility < visibility_threshold and right_foot_index.visibility < visibility_threshold:
        return False
    return True

def all_landmarks_visible(landmarks, visibility_threshold=0.5):
    for landmark in landmarks:
        if landmark.visibility < visibility_threshold:
            return False
    return True

class CVControl:
    def __init__(self):
        rospy.init_node('cv_control_test', anonymous=True)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.angle_publisher = rospy.Publisher("/robot_arm/angle", Float64, queue_size=1)
        self.status_publisher = rospy.Publisher("/cv_control/status", String, queue_size=10)  # 상태 퍼블리셔 추가
        self.processed_image_publisher = rospy.Publisher("/processed_image/compressed", CompressedImage, queue_size=1)  # 추가된 퍼블리셔
        self.camera_subscriber = rospy.Subscriber("/arm_camera/image/compressed", CompressedImage, self.camera_callback, queue_size=1)
        self.detection_subscriber = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.detection_callback, queue_size=1)
        self.bridge = CvBridge()
        self.largest_area_center = 320  # Image center horizontally
        self.current_landmarks = None
        self.is_facing_forward = False
        self.all_landmarks_visible = False
        self.detected = False
        self.largest_area = 0

    def detection_callback(self, data):
        self.process_detections(data)

    def process_detections(self, data):
        self.detected = False
        self.largest_area = 0
        self.largest_area_center = 320  # 이미지 수평 중심
        
        for box in data.bounding_boxes:
            if box.Class == "person" and box.probability > 0.2:
                center, area = self.calculate_box_center_and_area(box)
                if area > self.largest_area:
                    self.update_detected_person(center, area)

    def calculate_box_center_and_area(self, box):
        centerX = (box.xmin + box.xmax) // 2
        area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
        return centerX, area

    def update_detected_person(self, center, area):
        self.detected = True
        self.largest_area = area
        self.largest_area_center = center

    def rotate_target(self):
        target_center = 320

        if not self.all_landmarks_visible:
            angular_velocity = 0.002 * (target_center - self.largest_area_center)
            if angular_velocity==0:
                angular_velocity = 0.01
            linear_velocity_y = 0.15  # 원형 경로를 위한 속도 조정
            self.send_command(0, angular_velocity, linear_velocity_y)
        elif self.all_landmarks_visible:
            angular_velocity = 0.002 * (target_center - self.largest_area_center)
            
            if angular_velocity==0:
                angular_velocity = 0.01            
            linear_velocity_y = 0.15  # 원형 경로를 위한 속도 조정
            self.send_command(0, angular_velocity, linear_velocity_y)

    def move_at_step_thirty(self):
        print("Moving at step 30 until person is detected and facing forward")
        move_cmd = Twist()
        rospy.sleep(2)

        while not rospy.is_shutdown():
            if not self.is_person_detected():
                self.stop_robot()

            if self.is_person_detected() and not self.is_facing_forward:
                self.rotate_target()

            elif self.is_person_detected() and self.is_facing_forward:

                self.send_command(0, 0, 0)

            self.publish_status()
            rospy.sleep(1)

        self.send_command(0, 0, 0)
        print("Finished Step 30")

    def send_command(self, linearX_velocity, angular_velocity, linearY_velocity=0):
        command = Twist()
        command.linear.x = linearX_velocity
        command.angular.z = angular_velocity
        command.linear.y = linearY_velocity
        self.velocity_publisher.publish(command)
        rospy.loginfo(f"cmd_vel published - Linear Velocity: {command.linear.x}, Angular Velocity: {command.angular.z}, Linear Y: {command.linear.y}")

    def stop_robot(self):
        self.send_command(0, 0)

    def is_person_detected(self):
        # 사람 인식 여부를 반환하는 예제 메서드
        return self.current_landmarks is not None or self.detected

    def publish_status(self):
        # 정면 판단 여부와 all marker visible 여부를 문자열로 퍼블리시
        status_msg = f"Facing Forward: {self.is_facing_forward}, All Markers Visible: {self.all_landmarks_visible}, Person Detected: {self.detected}"
        self.status_publisher.publish(status_msg)
        rospy.loginfo(status_msg)

    def camera_callback(self, data):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image_rgb)

            if results.pose_landmarks:
                self.current_landmarks = results.pose_landmarks.landmark
                self.is_facing_forward = is_facing_forward(results.pose_landmarks.landmark, image.shape[1], image.shape[0])
                self.all_landmarks_visible = all_landmarks_visible(results.pose_landmarks.landmark)

                # Mediapipe 랜드마크를 이미지에 그리기
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            # 결과 이미지를 compressed image로 변환
            image_msg = self.bridge.cv2_to_compressed_imgmsg(image, "jpg")
            self.processed_image_publisher.publish(image_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridge error: {0}".format(e))

def main():
    ctrl = CVControl()
    rospy.sleep(2)  # 초기화 시간
    ctrl.move_at_step_thirty()

if __name__ == '__main__':
    main()
