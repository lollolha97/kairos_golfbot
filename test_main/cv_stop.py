import rospy
import cv2
import numpy as np
from detection_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Int32

class CVControl:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/follow/cmd_vel", Twist, queue_size=10)
        self.step_pub = rospy.Publisher("/step_control", Int32, queue_size=1)
        self.cmd = Twist()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.detection_callback, queue_size=1, buff_size=2**24)
        self.last_detected_positions = []  # 감지된 사람의 위치를 저장할 리스트
        self.last_move_time = time.time()  # 마지막으로 움직임이 감지된 시간
        self.step = 0  # 추가된 초기화 부분


    def detection_callback(self, data):
        big_area = 0
        big_center = 320  # 이미지의 가로 중앙 값
        detected = False
        current_time = time.time()

        for box in data.bounding_boxes:
            if box.Class == "person" and box.probability > 0.2:
                startX, startY, endX, endY = box.xmin, box.ymin, box.xmax, box.ymax
                rect_center = (startX + endX) // 2
                rect_area = (endX - startX) * (endY - startY)
                rospy.loginfo(f"Detected person: Probability {box.probability * 100:.2f}%, Box Coordinates: ({startX}, {startY}), ({endX}, {endY})")
                detected = True

                if rect_area > big_area:
                    big_area = rect_area
                    big_center = rect_center
                    # Update last detected positions and times
                    self.last_detected_positions.append((rect_center, current_time))
                    # Keep only the recent history
                    self.last_detected_positions = [pos for pos in self.last_detected_positions if current_time - pos[1] < 5]

        if detected and self.step==0:
            self.step_pub.publish(10)
            self.evaluate_stopped_condition()
            target_center = 320
            target_area = 100000 / 2
            dead_zone_width = 20  # 데드존 폭 설정 (20 픽셀)

            # 각속도 계산 로직 수정
            if abs(big_center - target_center) <= dead_zone_width:
                w = 0  # 데드존 내에 있으면 각속도 0
            else:
                kr = 0.002
                w = -kr * (big_center - target_center)  # 범위 바깥에 있을 때 각속도 조정

            kt = 0.0000045
            v = -kt * (big_area - target_area)
            maxv = 0.25
            v = max(-maxv, min(maxv, v))
            self.send_command(v, w)
        else:
            self.send_command(0, 0)

    def evaluate_stopped_condition(self):
        if len(self.last_detected_positions) > 1:
            for pos in self.last_detected_positions:
                if abs(pos[0] - self.last_detected_positions[0][0]) > 30:
                    return
            if time.time() - self.last_detected_positions[0][1] >= 3:
                print("정지했습니다.")
                self.step = 20
                self.step_pub.publish(20)  # 이제 이 상태에서 추가 작업을 수행하도록 두 번째 노드를 활성화

    


    def send_command(self, v, w):
        self.cmd.linear.x = v*3
        self.cmd.angular.z = w
        self.cmd_pub.publish(self.cmd)

def main():
    rospy.init_node('cv_control_compressed', anonymous=True)
    ctrl = CVControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
