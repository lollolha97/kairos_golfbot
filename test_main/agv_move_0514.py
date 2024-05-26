import rospy
import time
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import os
import cv2
import subprocess
import pygame
import mediapipe as mp
import numpy as np

class CVControl:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.step_publisher = rospy.Publisher("/step_control", Int32, queue_size=1)
        self.detection_subscriber = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.detection_callback, queue_size=1, buff_size=2**24)
        self.camera_subscriber = rospy.Subscriber("/arm_camera/image/compressed", CompressedImage, self.camera_callback, queue_size=1)
        self.step_subscriber = rospy.Subscriber("/step_control", Int32, self.step_callback)
        self.bridge = CvBridge()
        self.video_writer = None
        self.start_time = None
        self.last_detected_positions = []  # Detected positions of people
        self.last_move_time = time.time()
        self.step = 0
        self.video_duration = 10  # 비디오 녹화 시간 (초)
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.target_frame_y = None
        self.is_recording = False  # 녹화 상태 초기화

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
        print("Moving at step 30 for 3 seconds")
        rospy.sleep(1)
        move_cmd = Twist()
        move_cmd.linear.y = -0.6
        move_cmd.angular.z = 1.0
        self.velocity_publisher.publish(move_cmd)
        rospy.sleep(6)
        self.velocity_publisher.publish(Twist())  # Stop the robot after 3 seconds
        self.step = 40
        self.step_publisher.publish(40)
        self.move_at_step_forty_and_adjust_robot_arm()

    def move_at_step_forty_and_adjust_robot_arm(self):
        print("Adjusting robot arm at step 40")
        rospy.sleep(5)
        
        # MediaPipe를 사용하여 발목 위치 조정
        self.adjust_robot_arm_with_mediapipe()
        
        self.step = 50
        self.step_publisher.publish(50)
        self.start_video_recording()

    def adjust_robot_arm_with_mediapipe(self):
        # 로봇암 높이 조정 로직을 여기에 구현하십시오.
        print("Adjusting robot arm using MediaPipe")
        while self.step == 40:
            try:
                image_msg = rospy.wait_for_message("/arm_camera/image/compressed", CompressedImage, timeout=1)
                np_arr = np.frombuffer(image_msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                original_height, original_width = frame.shape[:2]

                if self.target_frame_y is None:
                    self.target_frame_y = int(original_height * 0.085)

                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = self.pose.process(frame_rgb)

                if results.pose_landmarks:
                    left_ankle = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_ANKLE]
                    right_ankle = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_ANKLE]
                    left_ankle_y = left_ankle.y * original_height
                    right_ankle_y = right_ankle.y * original_height

                    # 둘 중 더 낮은 위치의 발목 선택
                    target_ankle_y = max(left_ankle_y, right_ankle_y)

                    gap_y = self.target_frame_y - target_ankle_y

                    # 위치에 따라 AGV 이동
                    if abs(gap_y) > 20:  # Dead zone
                        linear_velocity = 0.1 if gap_y > 0 else -0.1
                    else:
                        linear_velocity = 0

                    self.send_command(linear_velocity, 0)

                else:
                    # 발목이 보이지 않으면 후진
                    self.send_command(-0.3, 0)

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
