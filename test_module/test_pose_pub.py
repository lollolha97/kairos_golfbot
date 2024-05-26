import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import mediapipe as mp

mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, model_complexity=1)

class PosePublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_subscriber = rospy.Subscriber("/arm_camera/image/compressed", CompressedImage, self.camera_callback, queue_size=1)
        self.landmarks_publisher = rospy.Publisher("/pose_landmarks", Float64MultiArray, queue_size=1)
        self.processed_image_publisher = rospy.Publisher("/processed_image/compressed", CompressedImage, queue_size=1)

    def camera_callback(self, data):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image_rgb)

            if results.pose_landmarks:
                landmarks_array = Float64MultiArray()
                for landmark in results.pose_landmarks.landmark:
                    landmarks_array.data.extend([landmark.x, landmark.y, landmark.z, landmark.visibility])
                self.landmarks_publisher.publish(landmarks_array)

                self.draw_landmarks(image, results.pose_landmarks.landmark, mp_pose.POSE_CONNECTIONS)

            image_msg = self.bridge.cv2_to_compressed_imgmsg(image, "jpg")
            self.processed_image_publisher.publish(image_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridge error: {0}".format(e))

    def draw_landmarks(self, image, landmarks, connections, visibility_threshold=0.5):
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

def main():
    rospy.init_node('pose_publisher', anonymous=True)
    PosePublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
