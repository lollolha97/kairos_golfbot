#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image  # Import Image instead of CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher_robot', anonymous=True)
        # Change the topic type to Image
        self.image_pub = rospy.Publisher('/arm_camera/image/raw', Image, queue_size=1)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            rospy.logerr("Cannot open camera.")
            exit()

    def publish_frames(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()

            if not ret:
                rospy.logerr("Cannot receive frame (stream end?). Exiting ...")
                break

            # Process only the lower quarter of the captured frame
            height = frame.shape[0]
            roi = frame[int(2 * height / 4):, :]  # Lower quarter

            try:
                # Convert the ROI to a ROS image message
                msg = self.bridge.cv2_to_imgmsg(roi, "bgr8")  # Use bgr8 as the encoding
                self.image_pub.publish(msg)
            except CvBridgeError as e:
                rospy.logerr(e)

            if cv2.waitKey(1) == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    camera_publisher = CameraPublisher()
    camera_publisher.publish_frames()
