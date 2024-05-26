import rospy
import time
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64, Bool
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from leg_tracker.msg import PersonArray
import os
import cv2
import subprocess
import pygame
import numpy as np
import mediapipe as mp
import math

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, model_complexity=1) #1

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
        self.leg_tracker_subscriber = rospy.Subscriber('/people_tracked', PersonArray, self.leg_tracker_callback)
        self.camera_subscriber = rospy.Subscriber("/arm_camera/image/compressed", CompressedImage, self.adjust_robot_arm_and_agv, queue_size=1)
        self.processed_image_publisher = rospy.Publisher("/processed_image/compressed", CompressedImage, queue_size=1)
        self.is_reset_subscriber = rospy.Subscriber("/is_reset", Bool, self.reset_callback)
        self.bridge = CvBridge()
        self.video_writer = None
        self.start_time = None
        self.last_detected_positions = []  # Detected positions of people
        self.last_move_time = time.time()
        self.step = 0
        self.video_duration = 10  # 비디오 녹화 시간 (초)
        self.is_facing_forward = False
        self.is_recording = False
        self.y_angle = 0
        self.target_frame_y = None
        self.interval = 0.5
        self.all_landmarks_visible = False
        self.detected = False
        self.largest_area = 0
        self.largest_area_center = 320  # Image center horizontally
        self.count = 0
        self.use_leg_tracker = False
        self.target_position = None
        self.follow_distance = 0.8
        self.linear_speed = 0.45
        self.max_angular_speed = 1.0
        self.rate = rospy.Rate(50)
        self.last_detection_time = time.time()  # 마지막 YOLO 검출 시간을 저장할 변수
        self.is_reset = False
        self.current_landmarks = None

    def reset_callback(self, msg):
        self.is_reset = msg.data
        if self.is_reset:
            print("System Restarting")
            self.step_publisher.publish(0)
            self.step = 0
            self.use_leg_tracker = False
            self.detected = False

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
        self.last_detected_positions = [pos for pos in self.last_detected_positions if current_time - pos[1] < 5]

    def is_person_detected(self):
        return self.detected

    def leg_tracker_callback(self, msg):
        if self.use_leg_tracker:
            min_distance = float('inf')
            closest_person = None

            for person in msg.people:
                distance = math.sqrt(person.pose.position.x**2 + person.pose.position.y**2)
                if distance < min_distance:
                    min_distance = distance
                    closest_person = person.pose.position

            self.target_position = closest_person
            rospy.loginfo(f"Leg tracker detected person at: {self.target_position}")

    def follow_person(self):
        if self.target_position:
            distance = math.sqrt(self.target_position.x**2 + self.target_position.y**2)
            angle_to_target = math.atan2(self.target_position.y, self.target_position.x)
            
            cmd_vel_msg = Twist()
            
            if distance > self.follow_distance:
                cmd_vel_msg.linear.x = distance / 20  # Move forward if the target is too far
            else:
                cmd_vel_msg.linear.x = 0.0  # Stop moving forward if within follow distance
            
            if abs(angle_to_target) > math.radians(30):
                cmd_vel_msg.angular.z = -0.6 * (angle_to_target / math.pi)  # Scale angular speed by angle
            else:
                cmd_vel_msg.angular.z = 0.0  # No rotation needed if within 5 degrees

            self.velocity_publisher.publish(cmd_vel_msg)
            rospy.loginfo(f"Following person with cmd_vel: linear.x={cmd_vel_msg.linear.x}, angular.z={cmd_vel_msg.angular.z}")
        else:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(cmd_vel_msg)

    def initiate_movement(self):
        self.step_publisher.publish(10)

    def move_toward_target(self):
        target_center = 320
        target_area = 35000 #50000
        dead_zone_width = 20
        if not self.is_person_detected():
            self.stop_robot()
            return
        if abs(self.largest_area_center - target_center) <= dead_zone_width:
            angular_velocity = 0
        else:
            angular_velocity = 0.002 * (target_center - self.largest_area_center)

        linear_velocity = -0.0000045 * (self.largest_area - target_area)
        linear_velocity = max(-0.25, min(0.25, linear_velocity))
        self.send_command(linear_velocity, angular_velocity)

    def stop_robot(self):
        self.send_command(0, 0)

    def evaluate_stopped_condition(self):
        if len(self.last_detected_positions) > 1 and all(abs(pos[0] - self.last_detected_positions[0][0]) <= 30 for pos in self.last_detected_positions):
            if time.time() - self.last_detected_positions[0][1] >= 3:
                self.step = 30
                self.step_publisher.publish(30)
                self.step_to_next()

    def step_to_next(self):
        print("Stopping.")
        self.step = 30
        self.step_publisher.publish(30)

    def rotate_target(self):
        target_center = 320

        if not self.all_landmarks_visible:
            angular_velocity = 0.001 * (target_center - self.largest_area_center)
            linear_velocity_y = 0.15  # 원형 경로를 위한 속도 조정
            self.send_command(0, angular_velocity, linear_velocity_y)
        elif self.all_landmarks_visible:
            angular_velocity = 0.001 * (target_center - self.largest_area_center)
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
                if self.all_landmarks_visible:
                    self.send_command(0, 0, 0)
                    rospy.sleep(0.5)
                    break
                else:
                    self.send_command(-0.1, 0, 0)
            rospy.sleep(1)
        self.send_command(0, 0, 0)
        print("Finished Step 30")
        self.send_command(0,0,0)
        self.step = 40
        self.step_publisher.publish(40)

    def adjust_robot_arm_and_agv(self, data):
        if self.step == 40:
            np_arr = np.frombuffer(data.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            original_height, original_width = frame.shape[:2]

            if self.target_frame_y is None:
                self.target_frame_y = int(original_height * 0.9)

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(frame_rgb)

            if results.pose_landmarks:
                gap_y = self.find_optimal_recording_angle(results.pose_landmarks.landmark, frame)
                self.find_foot_location(results.pose_landmarks.landmark, original_height, gap_y)

    def find_optimal_recording_angle(self, landmarks, frame):
        nose = landmarks[mp_pose.PoseLandmark.NOSE]
        nose_y = nose.y * frame.shape[0]

        target_nose_y = int(frame.shape[0] * 0.26)
        nose_x = int(nose.x * frame.shape[1])
        nose_y = int(nose.y * frame.shape[0])
        cv2.circle(frame, (nose_x, nose_y), 5, (0, 255, 0), -1)

        gap_y = target_nose_y - nose_y

        current_time = time.time()
        if current_time - self.last_move_time > self.interval:
            if abs(gap_y) > 30:
                if gap_y > 0:
                    self.y_angle -= 1
                else:
                    self.y_angle += 1
                self.y_angle = np.clip(self.y_angle, -20, 30)
                self.angle_publisher.publish(Float64(self.y_angle))
                self.last_move_time = current_time

        return gap_y

    def find_foot_location(self, landmarks, original_height, gap_y):
        left_foot_index = landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX]
        right_foot_index = landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX]
        left_knee = landmarks[mp_pose.PoseLandmark.LEFT_KNEE]
        right_knee = landmarks[mp_pose.PoseLandmark.RIGHT_KNEE]

        left_foot_y = left_foot_index.y * original_height
        right_foot_y = right_foot_index.y * original_height
        left_knee_y = left_knee.y * original_height
        right_knee_y = right_knee.y * original_height
        target_foot_y = max(left_foot_y, right_foot_y)
        gap_foot_y = self.target_frame_y - target_foot_y

        if left_foot_index.visibility < 0.5 or right_foot_index.visibility < 0.5:
            self.send_command(-0.1, 0)
            if left_knee_y.visibility<0.5 or right_knee_y.visibility<0.5:
                self.y_angle += 0.05
                self.y_angle = np.clip(self.y_angle, -30, 40)
                self.angle_publisher.publish(Float64(self.y_angle))
        else:
            if abs(gap_foot_y) > 30:
                linear_velocity = 0.1 if gap_foot_y > 0 else -0.1
            else:
                linear_velocity = 0
            self.send_command(linear_velocity, 0, 0)

        if abs(gap_y) <= 30 and abs(gap_foot_y) <= 30:
            self.step = 50
            self.step_publisher.publish(50)
            rospy.loginfo("Target position reached, publishing step 50.")
        else:
            rospy.loginfo(f"Gap Y: {gap_y}, Gap Foot Y: {gap_foot_y}")

    
    def start_video_recording(self):
        if not self.is_recording:
            print("Starting video recording at step 50")
            self.is_recording = True  # 녹화 상태 활성화
            pygame.mixer.init()
            sound_path = '/home/sang/start.mp3'
            pygame.mixer.music.load(sound_path)
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                rospy.sleep(0.1)

            self.start_time = rospy.get_time()
            video_path = '/home/sang/catkin_ws/src/Adeept-RaspberryYOLO-Follower/src/myagv_controller/video'
            if not os.path.exists(video_path):
                os.makedirs(video_path)
            video_file = os.path.join(video_path, '1.mp4')
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(video_file, fourcc, 10.0, (640, 480))

            rospy.Timer(rospy.Duration(self.video_duration), self.stop_video_recording, oneshot=True)

    def stop_video_recording(self, event=None):
        if self.is_recording:
            print("Video recording completed")
            self.video_writer.release()
            self.video_writer = None  # 비디오 라이터 객체 초기화
            self.is_recording = False  # 녹화 상태 업데이트

            pygame.mixer.music.load('/home/sang/clap.mp3')
            pygame.mixer.music.play()

            self.step = 60  # 다음 단계로 진행
            self.step_publisher.publish(60)
            self.analyze_video()
        else:
            print("Recording is not active, no need to stop.")

    def analyze_video(self):
        script_path = '/home/sang/catkin_ws/src/Adeept-RaspberryYOLO-Follower/src/myagv_controller/ai_coach.py'
        subprocess.run(['python3', script_path])

    def camera_callback(self, data):
        rospy.loginfo(f"STEP : {self.step} and is_recording {self.is_recording}, LIDAR : {self.use_leg_tracker}, CAM : {self.detected}")
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 추가된 부분 0524
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image_rgb)

            if self.step == 30:
                self.current_landmarks = results.pose_landmarks.landmark if results.pose_landmarks else None

            if self.step == 50 and self.is_recording:
                self.video_writer.write(image)

            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image_rgb)

            if results.pose_landmarks:
                self.current_landmarks = results.pose_landmarks.landmark
                self.is_facing_forward = is_facing_forward(results.pose_landmarks.landmark, image.shape[1], image.shape[0])
                self.all_landmarks_visible = all_landmarks_visible(results.pose_landmarks.landmark)

                # Mediapipe 랜드마크를 이미지에 그리기
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            image_msg = self.bridge.cv2_to_compressed_imgmsg(image, "jpg")
            self.processed_image_publisher.publish(image_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridge error: {0}".format(e))

    def send_command(self, linearX_velocity, angular_velocity, linearY_velocity=0):
        command = Twist()
        command.linear.x = linearX_velocity * 1
        command.angular.z = angular_velocity * 2
        command.linear.y = linearY_velocity
        self.velocity_publisher.publish(command)
        rospy.loginfo(f"cmd_vel published - Linear Velocity: {command.linear.x}, Angular Velocity: {command.angular.z}, Linear Y: {command.linear.y}")

    def run(self):
        rate = rospy.Rate(10)  # 10Hz 루프 속도 설정

        while not rospy.is_shutdown() and self.count < 4:
            rospy.loginfo(f"Current step: {self.step}, Use leg tracker: {self.use_leg_tracker}, Person detected: {self.is_person_detected()}")

            if self.is_reset:
                self.step_publisher.publish(100)
                self.step = 100
                self.use_leg_tracker = False
                self.detected = False
                self.count = 0
                self.is_reset = False

            if self.step == 0:
                self.step_publisher.publish(0)
                if self.is_person_detected():
                    self.initiate_movement()
                    self.step = 10

            elif self.step == 10:
                self.evaluate_stopped_condition()
                if self.use_leg_tracker:
                    self.follow_person()
                else:
                    self.move_toward_target()

            # elif self.step == 20:
            #     self.move_forward_briefly()

            elif self.step == 30:
                self.move_at_step_thirty()

            elif self.step == 40:
                self.adjust_robot_arm_and_agv()

            elif self.step == 50:
                if not self.is_recording:
                    rospy.sleep(3)  # 3초 대기
                    self.start_video_recording()

            elif self.step == 60:
                self.analyze_video()
                self.step = 40
                self.count += 1

            rate.sleep()

def main():
    rospy.init_node('cv_control_compressed', anonymous=True)
    ctrl = CVControl()
    ctrl.run()

if __name__ == '__main__':
    main()
