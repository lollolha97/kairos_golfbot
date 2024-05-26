#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def compressed_image_callback(msg):
    try:
        # Convert CompressedImage to a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        # Decode the numpy array to an OpenCV image
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        height, width, channels = cv_image.shape
        rospy.loginfo(f"Received image with size: {width}x{height}")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge error: {e}")
    except Exception as e:
        rospy.logerr(f"Error converting CompressedImage: {e}")

def main():
    rospy.init_node('compressed_image_size_subscriber', anonymous=True)
    rospy.Subscriber("/camera/image/compressed", CompressedImage, compressed_image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
