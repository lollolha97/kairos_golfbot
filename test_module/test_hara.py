import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np

class FaceDetectionNode:
    def __init__(self):
        # ROS 노드를 초기화합니다.
        rospy.init_node('face_detection_subscriber', anonymous=True)

        # 구독자와 퍼블리셔를 설정합니다.
        self.image_sub = rospy.Subscriber('/arm_camera/image/compressed', CompressedImage, self.image_callback)
        self.face_detected_pub = rospy.Publisher('/camera/face_detected', Bool, queue_size=10)
        self.result_image_pub = rospy.Publisher('/camera/result_image/compressed', CompressedImage, queue_size=10)

        # Haarcascade 얼굴 인식기를 로드합니다.
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_alt2.xml')

        # CvBridge를 초기화합니다.
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # 압축된 이미지를 디코딩합니다.
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 그레이스케일로 변환합니다.
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # 얼굴을 인식합니다.
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

            # 얼굴 인식 결과에 따라 변수를 설정합니다.
            face_detected = len(faces) > 0
            face_detected_msg = Bool(data=face_detected)
            self.face_detected_pub.publish(face_detected_msg)

            rospy.loginfo(f"Faces detected: {len(faces)}")

            # 인식된 얼굴에 사각형을 그립니다.
            for (x, y, w, h) in faces:
                cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)

            # 결과 이미지를 압축합니다.
            ret, buffer = cv2.imencode('.jpg', image)
            if not ret:
                rospy.logerr("Failed to encode result image")
                return

            # CompressedImage 메시지를 생성합니다.
            result_msg = CompressedImage()
            result_msg.header.stamp = rospy.Time.now()
            result_msg.format = "jpeg"
            result_msg.data = np.array(buffer).tobytes()

            # 결과 이미지 메시지를 발행합니다.
            self.result_image_pub.publish(result_msg)
            rospy.loginfo("Published result image")

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = FaceDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
