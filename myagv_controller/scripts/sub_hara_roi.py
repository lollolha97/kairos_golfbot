#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
import numpy as np
import rospkg
import os

rospack = rospkg.RosPack()
package_path = rospack.get_path('myagv_controller')

face_cascade_path = os.path.join(package_path, 'config', 'haarcascade_frontalface_alt2.xml')
face_cascade = cv2.CascadeClassifier(face_cascade_path)

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/arm_camera/image/compressed", CompressedImage, self.callback)
        # self.image_sub = rospy.Subscriber("/arm_camera/image/raw", Image, self.callback)

        self.image_pub = rospy.Publisher("/face_detection/image/compressed", CompressedImage, queue_size=10)
        self.status_pub = rospy.Publisher("/face_detection/status", Bool, queue_size=10)

    def callback(self, data):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=4, minSize=(30, 30))

            is_facing_front = False
            height = image.shape[0]
            upper_third = height // 2  # 화면 상단 1/3 지점 계산

            for (x, y, w, h) in faces:
                cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                if y < upper_third:  # 얼굴이 상단 1/3 부분에 있는지 확인
                    is_facing_front = True

            ret, jpeg = cv2.imencode('.jpg', image)
            if ret:
                compressed_image = CompressedImage()
                compressed_image.header = data.header
                compressed_image.format = "jpeg"
                compressed_image.data = jpeg.tobytes()
                self.image_pub.publish(compressed_image)

            self.status_pub.publish(is_facing_front)

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    image_subscriber = ImageSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
