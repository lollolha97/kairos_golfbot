import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import math
import mediapipe as mp

class Landmark:
    def __init__(self, x, y, z, visibility):
        self.x = x
        self.y = y
        self.z = z
        self.visibility = visibility

class PoseSubscriber:
    def __init__(self):
        self.landmarks_subscriber = rospy.Subscriber("/pose_landmarks", Float64MultiArray, self.landmarks_callback)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.current_landmarks = None
        self.mp_pose = mp.solutions.pose

    def landmarks_callback(self, data):
        self.current_landmarks = data.data
        self.process_landmarks()

    def process_landmarks(self):
        if self.current_landmarks:
            landmarks = self.convert_to_landmark_list(self.current_landmarks)
            if self.is_facing_forward(landmarks):
                self.send_command(0.5, 0)  # 예시: 전진 명령

    def convert_to_landmark_list(self, data):
        return [self.create_landmark(data[i:i+4]) for i in range(0, len(data), 4)]

    def create_landmark(self, data):
        return Landmark(data[0], data[1], data[2], data[3])

    def is_facing_forward(self, landmarks):
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
        left_eye = landmarks[self.mp_pose.PoseLandmark.LEFT_EYE.value]
        right_eye = landmarks[self.mp_pose.PoseLandmark.RIGHT_EYE.value]

        ls = (left_shoulder.x, left_shoulder.y)
        rs = (right_shoulder.x, right_shoulder.y)
        le = (left_eye.x, left_eye.y)
        re = (right_eye.x, right_eye.y)

        eye_line = abs(le[1] - re[1])
        shoulder_line = abs(ls[1] - rs[1])

        eye_distance = self.calculate_distance(le, re)
        shoulder_distance = self.calculate_distance(ls, rs)

        face_size = eye_distance
        y_threshold = face_size * 0.2
        distance_ratio_threshold = 0.7

        if eye_line < y_threshold and shoulder_line < y_threshold and shoulder_distance / eye_distance > distance_ratio_threshold:
            return True
        return False

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def send_command(self, linear_velocity, angular_velocity):
        command = Twist()
        command.linear.x = linear_velocity
        command.angular.z = angular_velocity
        self.velocity_publisher.publish(command)

def main():
    rospy.init_node('pose_subscriber', anonymous=True)
    PoseSubscriber()
    rospy.spin()

if __name__ == '__main__':
    main()
