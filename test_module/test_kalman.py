#!/usr/bin/env python3
import rospy
from detection_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class KalmanFilter:
    def __init__(self, dt=0.1, u_x=0, u_y=0, std_acc=1, std_meas=0.5):
        self.dt = dt
        self.u = np.matrix([[u_x], [u_y]])

        self.A = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        self.B = np.matrix([[(self.dt**2)/2, 0],
                            [0, (self.dt**2)/2],
                            [self.dt, 0],
                            [0, self.dt]])

        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        self.Q = np.matrix([[self.dt**4/4, 0, self.dt**3/2, 0],
                            [0, self.dt**4/4, 0, self.dt**3/2],
                            [self.dt**3/2, 0, self.dt**2, 0],
                            [0, self.dt**3/2, 0, self.dt**2]]) * std_acc**2

        self.R = np.matrix([[std_meas, 0],
                            [0, std_meas]])

        self.P = np.eye(self.A.shape[1])

        self.x = np.matrix([[0], [0], [0], [0]])

    def predict(self):
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[0:2]

    def update(self, z):
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.H.shape[1])
        self.P = (I - np.dot(K, self.H)) * self.P
        return self.x[0:2]

class BoundingBoxEKF:
    def __init__(self):
        rospy.init_node('bounding_box_ekf', anonymous=True)
        self.bridge = CvBridge()
        self.kalman_filters = {}

        self.bbox_sub = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.bbox_callback)
        self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback)
        self.bbox_pub = rospy.Publisher("/bounding_boxes_ekf", BoundingBoxes, queue_size=10)
        self.image_pub = rospy.Publisher("/yolov5/image/compressed", CompressedImage, queue_size=10)

        self.current_image = None

    def generate_key(self, box):
        # Generate a unique key for each bounding box based on class and probability
        return f"{box.Class}_{box.probability:.2f}_{box.xmin}_{box.ymin}_{box.xmax}_{box.ymax}"

    def bbox_callback(self, data):
        smoothed_boxes = BoundingBoxes()
        smoothed_boxes.header = data.header

        for box in data.bounding_boxes:
            centerX = (box.xmin + box.xmax) // 2
            centerY = (box.ymin + box.ymax) // 2
            width = box.xmax - box.xmin
            height = box.ymax - box.ymin

            key = self.generate_key(box)
            if key not in self.kalman_filters:
                self.kalman_filters[key] = KalmanFilter()

            kalman = self.kalman_filters[key]
            z = np.matrix([[centerX], [centerY]])
            smoothed_center = kalman.update(z)

            smoothed_box = BoundingBox()
            smoothed_box.Class = box.Class
            smoothed_box.probability = box.probability
            smoothed_box.xmin = int(smoothed_center[0] - width / 2)
            smoothed_box.ymin = int(smoothed_center[1] - height / 2)
            smoothed_box.xmax = int(smoothed_center[0] + width / 2)
            smoothed_box.ymax = int(smoothed_center[1] + height / 2)

            smoothed_boxes.bounding_boxes.append(smoothed_box)

        self.bbox_pub.publish(smoothed_boxes)
        self.visualize(smoothed_boxes)

    def image_callback(self, data):
        try:
            self.current_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: {0}".format(e))

    def visualize(self, smoothed_boxes):
        if self.current_image is None:
            return

        vis_image = self.current_image.copy()
        for box in smoothed_boxes.bounding_boxes:
            cv2.rectangle(vis_image, (box.xmin, box.ymin), (box.xmax, box.ymax), (0, 255, 0), 2)
            cv2.putText(vis_image, box.Class, (box.xmin, box.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        try:
            image_msg = self.bridge.cv2_to_compressed_imgmsg(vis_image, "jpg")
            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: {0}".format(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = BoundingBoxEKF()
    node.run()
