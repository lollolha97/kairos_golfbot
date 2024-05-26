import rospy
import time
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import os
import cv2
import subprocess
import pygame
import numpy as np
import mediapipe as mp
import math

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
        self.angle_publisher = rospy.Publisher("/robot_arm/angle", Float64, queue_size=1)
        self.step_publisher = rospy.Publisher("/step_control", Int32, queue_size=1)
        self.detection_subscriber = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.detection_callback, queue_size=1, buff_size=2**24)
        self.camera_subscriber = rospy.Subscriber("/arm_camera/image/compressed", CompressedImage, self.camera_callback, queue_size=1)
        self.processed_image_publisher = rospy.Publisher("/processed_image/compressed", CompressedImage, queue_size=1)
        self.bridge = CvBridge()
        self.video_writer = None
        self.start_time = None
        self.last_detected_positions = []  # Detected positions of people
        self.last_move_time = time.time()
        self.step = 0
        self.video_duration = 10  # 비디오 녹화 시간 (초)
        self.is_facing_forward = False
        self.y_angle = 0
        self.target_frame_y = None
        self.interval = 0.5

    def step_callback(self, data):
        self.step = data.data

    def detection_callback(self, data):
        self.process_detections(data)
        if self.is_person_detected() and self.step == 0:
            self.initiate_movement()
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

    def initiate_movement(self):
        self.step_publisher.publish(10)
        self.evaluate_stopped_condition()
        self.move_toward_target()

    def move_toward_target(self):
        target_center = 320
        target_area = 50000
        dead_zone_width = 20

        if abs(self.largest_area_center - target_center) <= dead_zone_width:
            angular_velocity = 0
        else:
            angular_velocity = -0.002 * (self.largest_area_center - target_center)

        linear_velocity = -0.0000045 * (self.largest_area - target_area)
        linear_velocity = max(-0.25, min(0.25, linear_velocity))
        self.send_command(linear_velocity, angular_velocity)

    def stop_robot(self):
        self.send_command(0, 0)

    def evaluate_stopped_condition(self):
        if len(self.last_detected_positions) > 1 and all(abs(pos[0] - self.last_detected_positions[0][0]) <= 30 for pos in self.last_detected_positions):
            if time.time() - self.last_detected_positions[0][1] >= 3:
                self.step_to_next()

    def step_to_next(self):
        print("Stopping.")
        self.step = 20
        self.step_publisher.publish(20)
        self.move_forward_briefly()

    def move_forward_briefly(self):
        rospy.sleep(1)
        move_cmd = Twist()
        move_cmd.linear.x = 0.3
        self.velocity_publisher.publish(move_cmd)
        rospy.sleep(2)
        self.velocity_publisher.publish(Twist())
        self.step = 30
        self.step_publisher.publish(30)
        self.move_at_step_thirty()

    def move_at_step_thirty(self):
        print("Moving at step 30 until person faces forward")
        move_cmd = Twist()
        move_cmd.angular.z = 1.0  # Rotate the robot
        while not self.is_facing_forward and not rospy.is_shutdown():
            self.velocity_publisher.publish(move_cmd)
            rospy.sleep(0.1)
        self.velocity_publisher.publish(Twist())  # Stop the robot
        self.step = 40
        self.step_publisher.publish(40)
        self.adjust_robot_arm_and_agv()

    def adjust_robot_arm_and_agv(self):
        print("Adjusting robot arm at step 40")
        while self.step == 40:
            try:
                image_msg = rospy.wait_for_message("/arm_camera/image/compressed", CompressedImage, timeout=1)
                np_arr = np.frombuffer(image_msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                original_height, original_width = frame.shape[:2]

                if self.target_frame_y is None:
                    self.target_frame_y = int(original_height * 0.9)

                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = pose.process(frame_rgb)

                if results.pose_landmarks:
                    nose = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]
                    left_foot_index = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_FOOT_INDEX]
                    right_foot_index = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX]
                    nose_y = nose.y * frame.shape[0]

                    target_nose_y = int(frame.shape[0] * 0.26)
                    nose_x = int(nose.x * frame.shape[1])
                    nose_y = int(nose.y * frame.shape[0])
                    cv2.circle(frame, (nose_x, nose_y), 5, (0, 255, 0), -1)

                    gap_y = target_nose_y - nose_y

                    current_time = time.time()
                    if current_time - self.last_move_time > self.interval:
                        if abs(gap_y) > 50:
                            if gap_y > 0:
                                self.y_angle -= 1
                                print("각도 증가")
                            else:
                                self.y_angle += 1
                                print("각도 감소")

                            self.y_angle = np.clip(self.y_angle, -30, 40)
                            self.angle_publisher.publish(Float64(self.y_angle))
                            self.last_move_time = current_time

                    left_foot_y = left_foot_index.y * original_height
                    right_foot_y = right_foot_index.y * original_height
                    target_foot_y = max(left_foot_y, right_foot_y)
                    gap_foot_y = self.target_frame_y - target_foot_y

                    if left_foot_index.visibility < 0.5 or right_foot_index.visibility < 0.5:
                        self.send_command(-0.3, 0)
                    else:
                        if abs(gap_foot_y) > 40:
                            linear_velocity = 0.1 if gap_foot_y > 0 else -0.1
                        else:
                            linear_velocity = 0

                        self.send_command(linear_velocity, 0)

                    if abs(gap_y) <= 50 and abs(gap_foot_y) <= 40:
                        self.step = 50
                        self.step_publisher.publish(50)
                        rospy.loginfo("Target position reached, publishing step 50.")
                        break

                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                _, buffer = cv2.imencode('.jpg', frame)
                image_msg = CompressedImage()
                image_msg.header.stamp = rospy.Time.now()
                image_msg.format = 'jpeg'
                image_msg.data = np.array(buffer).tobytes()
                self.processed_image_publisher.publish(image_msg)

            except rospy.ROSException as e:
                print("Waiting for image message timed out: ", e)

    def start_video_recording(self):
        print("Starting video recording at step 50")
        self.is_recording = True  # 녹화 상태 활성화
        pygame.mixer.init()
        sound_path = '/home/sang/start.mp3'
        pygame.mixer.music.load(sound_path)
        pygame.mixer.music.play()

        # 사운드 재생이 끝날 때까지 대기
        while pygame.mixer.music.get_busy():
            rospy.sleep(0.1)

        self.start_time = rospy.get_time()
        video_path = '/home/sang/catkin_ws/src/Adeept-RaspberryYOLO-Follower/src/myagv_controller/video'
        if not os.path.exists(video_path):
            os.makedirs(video_path)
        video_file = os.path.join(video_path, '../video/1.mp4')
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(video_file, fourcc, 10.0, (640, 480))

    def camera_callback(self, data):
        if self.step == 50 and self.is_recording:
            current_time = rospy.get_time()
            if current_time - self.start_time < self.video_duration:
                try:
                    cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
                    self.video_writer.write(cv_image)
                except CvBridgeError as e:
                    print(e)
            else:
                # 녹화 시간 초과 시 녹화 중지
                self.stop_video_recording()
                pygame.mixer.music.load('/home/sang/start.mp3')
                pygame.mixer.music.play()

        elif self.step == 30:
            try:
                np_arr = np.frombuffer(data.data, np.uint8)
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

            except CvBridgeError as e:
                rospy.logerr("CvBridge 에러: {0}".format(e))

    def stop_video_recording(self):
        if self.is_recording:
            print("Video recording completed")
            self.video_writer.release()
            self.video_writer = None  # 비디오 라이터 객체 초기화
            self.is_recording = False  # 녹화 상태 업데이트
            self.step = 60  # 다음 단계로 진행
            self.step_publisher.publish(60)
            self.analyze_video()
        else:
            print("Recording is not active, no need to stop.")

    def analyze_video(self):
        script_path = '/home/sang/catkin_ws/src/Adeept-RaspberryYOLO-Follower/src/myagv_controller/ai_coach.py'  # 실행하고 싶은 스크립트의 경로
        subprocess.run(['python3', script_path])  # Python 3.x에서 파이썬 파일 실행

    def send_command(self, linear_velocity, angular_velocity):
        command = Twist()
        command.linear.x = linear_velocity * 1.5
        command.angular.z = angular_velocity * 2
        self.velocity_publisher.publish(command)
        rospy.loginfo(f"cmd_vel published - Linear Velocity: {command.linear.x}, Angular Velocity: {command.angular.z}")

def main():
    rospy.init_node('cv_control_compressed', anonymous=True)
    ctrl = CVControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
