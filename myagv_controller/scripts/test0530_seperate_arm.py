#!/usr/bin/env python3
import rospy
import time
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray, Int32, Bool
import threading
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
import rospkg

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, model_complexity=1)

bridge = CvBridge()
rospack = rospkg.RosPack()

speed_factor = 1

''' File Path '''
package_path = rospack.get_path('myagv_controller')
parent_path = os.path.dirname(package_path)
script_path = os.path.join(package_path, 'scripts', 'ai_coach.py')
video_path = os.path.join(parent_path, 'flask','flask5_design_iphone','static','videos')
start_mp3_path = os.path.join(parent_path, 'mp3', 'start.mp3')
end_mp3_path = os.path.join(parent_path, 'mp3', 'clap.mp3')

# Utility Functions
def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

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

# Main Control Class
class CVControl:
    def __init__(self):
        rospy.init_node('cv_control_compressed', anonymous=True)
        
        # Publishers and Subscribers
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=5) #quesize = 10
        self.coords_publisher = rospy.Publisher("/arm_camera/coords", Float64MultiArray, queue_size=10)
        self.angle_publisher = rospy.Publisher("/arm_camera/angle", Float64, queue_size=5)
        self.step_publisher = rospy.Publisher("/step_control", Int32, queue_size=1)
        self.detection_subscriber = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.detection_callback, queue_size=1, buff_size=2**24) #1 0529 # buff_size 추가 # 추가한 이유는 메시지 크기가 크기 때문에
        self.leg_tracker_subscriber = rospy.Subscriber('/people_tracked', PersonArray, self.leg_tracker_callback)
        self.camera_subscriber = rospy.Subscriber("/arm_camera/image/compressed", CompressedImage, self.camera_callback, queue_size=10)
        self.processed_image_publisher = rospy.Publisher("/processed_image/compressed", CompressedImage, queue_size=1)
        self.is_reset_subscriber = rospy.Subscriber("/is_reset", Bool, self.reset_callback)
        self.step_subscriber = rospy.Subscriber("/step_control", Int32, self.step_callback)
        self.front_face_detected = rospy.Subscriber("/face_detection/status", Bool, self.front_face_callback, queue_size=10)
        self.is_robotarm_moving = rospy.Subscriber("/is_arm_moving", Bool, self.is_moving_callback, queue_size=1)
        
        ''' 기본 설정 params '''
        self.bridge = CvBridge()
        self.video_writer = None
        self.start_time = None
        self.end_time = None
        self.rate = rospy.Rate(10) #30Hz 녹화시 부드러움, 추적 시 문제 발생
        self.current_image = None  # 현재 이미지를 저장할 변수

        self.previous_step = 100

        ''' 감지 관련 params '''
        self.last_detected_positions = []  # Detected positions of people
        self.last_move_time = time.time()
        self.last_detection_time = time.time()  # 마지막 YOLO 검출 시간을 저장할 변수
        self.largest_area = 0
        self.largest_area_center = 320  # Image center horizontally - callback에서 검출되지만 검출되지 않을 때 튐 방지용
        self.detected = False
        self.all_landmarks_visible = False
        self.current_landmarks = None

        '''mediapipe 관련 params'''
        self.target_frame_y = None
        self.y_angle = 0
        self.interval = 0.5

        ''' 비디오 녹화 관련 params '''
        self.video_duration = 10  # 비디오 녹화 시간 (초)
        self.is_recording = False

        ''' 로봇 제어 관련 params '''
        self.step = 0
        # self.previous_step = None  # 이전 스텝을 저장할 변수
        self.count = 0
        self.is_reset = False
        self.use_leg_tracker = False
        self.target_position = None
        self.follow_distance = 0.8
        self.frames_with_left_hand_condition = 0  # 왼손 조건을 충족하는 프레임 수를 저장할 변수
        self.largest_area_center = 0  # 예제에서는 초기값 설정
        self.last_rotation_time = rospy.get_time()
        self.is_rotating = False
        self.is_robotarm_moving = False

        # self.coords_1 = [-165.30,92.40,397.00,-90.00,2.28,96.67]
        # self.coords_2 = [-152.70,0.10,522.50,-90.53,3.91,83.03]
        # self.coords_3 = [-153.00,-142.40,387.70,-89.91,-2.54,88.15]
        # self.coords_4 = [-152.70,0.10,522.50,-90.53,3.91,83.03]

        ''' 얼굴 인식 관련 params (0526 추가 - haracascade 정면인지 로직) '''
        self.front_face = False
        self.front_face_history = []

        ''' YOLO 관련 params (0528 추가) '''
        self.target_center = 320
        self.dead_zone_width = 100

    ''' AGV 직접 제어 함수'''    
    # Section: Command Sending
    def send_command(self, linearX_velocity, angular_velocity, linearY_velocity=0):
        command = Twist()
        command.linear.x = linearX_velocity * 1 * speed_factor
        command.angular.z = angular_velocity * 2 * speed_factor
        command.linear.y = linearY_velocity
        self.velocity_publisher.publish(command)
        rospy.loginfo(f"cmd_vel published - Linear Velocity: {command.linear.x}, Angular Velocity: {command.angular.z}, Linear Y: {command.linear.y}")
    
    def stop_robot(self):
        self.send_command(0, 0)

    ''' Robot Arm 직접 제어 함수'''    
    # Section: Arm Movement
    def move_robot_arm(self, angle):
        self.angle_publisher.publish(Float64(angle))
        
    def send_coords(self, coords):
        msg = Float64MultiArray(data=coords)
        self.coords_publisher.publish(msg)

    def is_moving_callback(self, msg):
        self.is_moving_callback = msg.data

    ''' Step 직접 제어 함수'''    
    def update_step(self, step):
        self.step = step
        self.step_publisher.publish(step)

    ''' Step Subscribe Callback'''    
    # Section: Step Management
    def step_callback(self, msg):
        self.step = msg.data

        if self.previous_step != self.step:
            self.previous_step = self.step
            self.play_mp3_and_log(self.step)  # 새로운 스텝을 Sub하면 MP3 재생

    ''' mp3 Play 관련 함수'''

    def play_mp3_and_log(self, step):
        mp3_path = os.path.join(parent_path, 'mp3', f'step{step}.mp3')

        if mp3_path and os.path.exists(mp3_path):
            print(f"Playing MP3 for step {step}: {mp3_path}")
            pygame.mixer.init()
            pygame.mixer.music.load(mp3_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                rospy.sleep(0.1)
        else:
            print(f"No MP3 file found for step {step}")

    # Section: Person Detection
    def detection_callback(self, data):
        self.process_detections(data)

        if self.detected:
            self.last_detection_time = time.time()  # 마지막 검출 시간 업데이트
            self.use_leg_tracker = False

        elif time.time() - self.last_detection_time > 3.0:
            self.use_leg_tracker = True

    def evaluate_stopped_condition(self):
        if len(self.last_detected_positions) > 1 and all(abs(pos[0] - self.last_detected_positions[0][0]) <= 30 for pos in self.last_detected_positions):
            if time.time() - self.last_detected_positions[0][1] >= 3:
                # self.step_to_next()
                self.update_step(30)

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
        self.last_detected_positions = [pos for pos in self.last_detected_positions if current_time - pos[1] < 5]

    def is_person_detected(self):
        return self.detected

    # Section: Leg Tracker
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
                        
            if distance > self.follow_distance:
                linear_x = distance / 20  # Move forward if the target is too far
            else:
                linear_x = 0.0  # Stop moving forward if within follow distance
            
            if abs(angle_to_target) > math.radians(30):
                angular_z = -0.3 * (angle_to_target / math.pi)  # Scale angular speed by angle
            else:
                angular_z = 0.0  # No rotation needed if within 5 degrees

            self.send_command(linear_x, angular_z)
            rospy.loginfo(f"Following person with cmd_vel: linear.x={linear_x}, angular.z={angular_z}")
        else:
            cmd_vel_msg = Twist()
            self.send_command(0, 0)

    # Section: Robot Movement
    def move_toward_target(self):
        target_area = 35000 #150000
        target_area_threshold = 1500

        if not self.is_person_detected():
            self.stop_robot()
            return
        if abs(self.largest_area - target_area) <= target_area_threshold:
            self.evaluate_stopped_condition()

        if abs(self.largest_area_center - self.target_center) <= self.dead_zone_width:
            angular_velocity = 0
        else:
            angular_velocity = 0.0006 * 2 * (self.target_center - self.largest_area_center) # 0.0006

        linear_velocity = -0.000002 * (self.largest_area - target_area) *2 # -0.000002
        linear_velocity = max(-0.25, min(0.25, linear_velocity))
        self.send_command(linear_velocity, angular_velocity)

    # def rotate_target(self):
    #     total_width = 1280  # 총 너비를 설정
    #     left_bound = total_width // 4
    #     right_bound = 3 * (total_width // 4)
    #     target_center = 320
    #     angular_velocity = 0.1
    #     current_time = rospy.get_time()

    #     if self.is_rotating:
    #         if current_time - self.last_rotation_time >= 0.2:
    #             self.is_rotating = False
    #             self.last_rotation_time = current_time
    #             self.stop_robot()
    #             return
    #     else:
    #         if current_time - self.last_rotation_time >= 0.1:
    #             self.is_rotating = True
    #             self.last_rotation_time = current_time

    #     if self.is_rotating:
    #         if self.largest_area_center < left_bound:  # 왼쪽 영역
    #             angular_velocity = 0.1  # 오른쪽으로 회전
    #             self.send_command(0, angular_velocity, 0)
    #         elif self.largest_area_center > right_bound:  # 오른쪽 영역
    #             angular_velocity = -0.1  # 왼쪽으로 회전
    #             self.send_command(0, angular_velocity, 0)
    #         else:  # 중앙 영역
    #             if target_center - self.largest_area_center > 0:
    #                 angular_velocity = -0.1
    #                 linear_velocity_y = 0.2  # 원형 경로를 위한 속도 조정
    #                 self.send_command(0, angular_velocity, linear_velocity_y)
    #             else:
    #                 angular_velocity = 0.1
    #                 self.send_command(0, angular_velocity, 0)

    # def rotate_target(self):
    #     total_width = 1280  # 총 너비를 설정
    #     left_bound = total_width // 3
    #     right_bound = 2 * (total_width // 3)
    #     target_center = 640
    #     angular_velocity = 0.1
        
    #     if self.largest_area_center < left_bound:  # 왼쪽 영역
    #         angular_velocity = 0.1  # 오른쪽으로 회전
    #         self.send_command(0, angular_velocity, 0)
    #     elif self.largest_area_center > right_bound:  # 오른쪽 영역
    #         angular_velocity = -0.1  # 왼쪽으로 회전
    #         self.send_command(0, angular_velocity, 0)
    #     else:  # 중앙 영역
    #         if target_center - self.largest_area_center > 0:
    #             angular_velocity = -0.1
    #             linear_velocity_y = 0.2  # 원형 경로를 위한 속도 조정
    #             self.send_command(0, angular_velocity, linear_velocity_y)
    #         else:
    #             angular_velocity = 0.1
    #             self.send_command(0, angular_velocity, 0)

    def rotate_target(self):
        angular_velocity = 0.001 * (self.target_center - self.largest_area_center)
        if angular_velocity == 0:
            angular_velocity = -0.01
        linear_velocity_y = 0.12 # 0.12
        self.send_command(0, angular_velocity, linear_velocity_y)

    # def rotate_target(self):
    #     angular_velocity = 0.001 * (self.target_center - self.largest_area_center)
        
    #     # 추가 로깅
    #     rospy.loginfo(f"Target center: {self.target_center}, Largest area center: {self.largest_area_center}, Calculated angular_velocity: {angular_velocity}")
        
    #     # 작은 값으로 회전을 유도
    #     if abs(angular_velocity) < 0.01:
    #         angular_velocity = 0.01 if angular_velocity == 0 else angular_velocity / abs(angular_velocity) * 0.01

    #     # 각도 속도를 제한 (예: -0.5 ~ 0.5 사이로)
    #     angular_velocity = max(-0.5, min(0.5, angular_velocity))
        
    #     linear_velocity_y = 0.12
    #     self.send_command(0, angular_velocity, linear_velocity_y)
    #     rospy.loginfo(f"Sending command: linear.y={linear_velocity_y}, angular.z={angular_velocity}")


    def move_forward_briefly(self):
        # Logic for brief forward movement
        pass

    # Section: Step Specific Movements
    def rearrangement(self):
        update_step = 40

        if not self.is_person_detected():
            self.stop_robot()

        if abs(self.largest_area_center - self.target_center) <= self.dead_zone_width:
            angular_velocity = 0
        else:
            angular_velocity = 0.0006 * (self.target_center - self.largest_area_center)

        self.send_command(0, angular_velocity)
        self.update_step(update_step)

    def move_at_step_thirty(self):
        update_step = 35
        print("Moving at step 30 until person is detected and facing forward")
        rospy.sleep(2)
        while not rospy.is_shutdown():
            if not self.is_person_detected():
                self.stop_robot()
            if self.is_person_detected() and not self.detected_front_face():
                self.rotate_target()
            elif self.is_person_detected() and self.detected_front_face():
                self.stop_robot()
                rospy.sleep(0.5)
                break
            rospy.sleep(1)
        print("Finished Step 30")
        self.update_step(update_step)

    def adjust_robot_arm_and_agv(self): 
        print("Moving at step 50")
        if self.current_landmarks:
            original_height, original_width = self.current_image.shape[:2]
            if self.target_frame_y is None:
                self.target_frame_y = int(original_height * 0.9)

            gap_y = self.find_optimal_recording_angle(self.current_landmarks, self.current_image)
            self.find_foot_location(self.current_landmarks, original_height, gap_y)

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
                self.y_angle = np.clip(self.y_angle, -30, 40)
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
            if left_knee.visibility < 0.5 or right_knee.visibility < 0.5:
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
            self.update_step(50)
            rospy.loginfo("Target position reached, publishing step 50.")

        else:
            rospy.loginfo(f"Gap Y: {gap_y}, Gap Foot Y: {gap_foot_y}")
            
    def start_video_recording(self):
        if not self.is_recording:
            print("Starting video recording at step 50")
            self.is_recording = True  # 녹화 상태 활성화
            pygame.mixer.init()
            pygame.mixer.music.load(start_mp3_path)
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                rospy.sleep(0.1)

            self.start_time = rospy.get_time()
            if not os.path.exists(video_path):
                os.makedirs(video_path)
            video_file = os.path.join(video_path, '1.mp4')
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(video_file, fourcc, 30.0, (640, 480))

            rospy.Timer(rospy.Duration(self.video_duration), self.stop_video_recording, oneshot=True)

    def stop_video_recording(self, event=None):
        if self.is_recording:
            print("Video recording completed")
            self.video_writer.release()
            self.video_writer = None  # 비디오 라이터 객체 초기화
            self.is_recording = False  # 녹화 상태 업데이트

            pygame.mixer.music.load(end_mp3_path)
            pygame.mixer.music.play()

            self.update_step(60)
        else:
            print("Recording is not active, no need to stop.")


    def analyze_video(self):
        # def move_robot_arm_back_and_forth():
        #     for i in range(2):  # 2회 반복 (필요에 따라 조정 가능)
        #         print('시작시작시작시작시작시작시작시작시작시작시작시작시작시작시작')
        #         self.send_coords(self.coords_1)
        #         self.send_coords(self.coords_2)
        #         self.send_coords(self.coords_3)
        #         self.send_coords(self.coords_4)
                   
        # import threading
        # arm_movement_thread = threading.Thread(target=move_robot_arm_back_and_forth)
        # arm_movement_thread.start()
        subprocess.run(['python3', script_path])
        # arm_movement_thread.join()

    # Section: Callbacks
    def camera_callback(self, data):
        rospy.loginfo(f"STEP : {self.step} and is_recording {self.is_recording}, LIDAR : {self.use_leg_tracker}, CAM : {self.detected}")
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.current_image = image  # 현재 이미지를 저장

            # # 마커가 포함되지 않은 원본 영상 저장
            # original_image = image.copy()

            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image_rgb)

            if self.step == 50 and self.is_recording:
                if self.video_writer is not None:
                    self.video_writer.write(image)  # 마커가 없는 원본 영상 저장

            if results.pose_landmarks:
                self.current_landmarks = results.pose_landmarks.landmark
                # Mediapipe 랜드마크를 이미지에 그리기
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            image_msg = self.bridge.cv2_to_compressed_imgmsg(image, "jpg")
            self.processed_image_publisher.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: {0}".format(e))
        except Exception as e:
            rospy.logerr(f"Unexpected error: {str(e)}")

    def front_face_callback(self, msg):
        self.front_face = msg.data
        self.front_face_history.append(self.front_face)
        if len(self.front_face_history) > 30:
            self.front_face_history.pop(0)

    def detected_front_face(self):
        if self.front_face_history.count(True) > 15:
            return True
        return False

    def reset_callback(self, msg):
        self.is_reset = msg.data
        if self.is_reset:
            print("System Restarting")
            self.update_step(0)
            self.use_leg_tracker = False
            self.detected = False

    # Section: Main Loop
    def run(self):
        while not rospy.is_shutdown() and self.count < 4:
            rospy.loginfo(f"Current step: {self.step}, Use leg tracker: {self.use_leg_tracker}, Person detected: {self.is_person_detected()}")
            
            if self.is_reset:
                # self.update_step(100)
                self.use_leg_tracker = False
                self.detected = False
                # self.count = 0
                # self.is_reset = False

            if self.step == 0:
                self.is_reset = False
                self.step_publisher.publish(0)
                # self.move_robot_arm(-5)
                if self.is_person_detected():
                    self.update_step(10)

            elif self.step == 10:
                if not self.is_robotarm_moving:
                    if self.use_leg_tracker:
                        self.follow_person()
                    else:
                        self.move_toward_target()

            # elif self.step == 20:
            #     self.move_forward_briefly()

            elif self.step == 30:
                if not self.is_robotarm_moving:
                    self.move_at_step_thirty()

            elif self.step == 35:
                self.rearrangement()

            elif self.step == 40:
                self.adjust_robot_arm_and_agv()

            elif self.step == 50:
                if not self.is_recording:
                    self.stop_robot()
                    # 여유시간 1.5초 부여
                    rospy.sleep(2)
                    self.start_video_recording()

            elif self.step == 60:
                self.analyze_video()
                self.count += 1
                # rospy.sleep(10)
                self.update_step(100)
            
            elif self.step == 100 and self.is_reset == False:
                self.is_reset = True
                self.stop_robot()
                self.move_robot_arm(-5)

            self.rate.sleep()

def main():
    ctrl = CVControl()
    rospy.sleep(2)
    ctrl.run()

if __name__ == '__main__':
    main()
