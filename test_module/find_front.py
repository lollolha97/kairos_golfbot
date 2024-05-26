import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class RobotController:
    def __init__(self):
        self.pub = rospy.Publisher('/find/cmd_vel', Twist, queue_size=10)
        self.step_pub = rospy.Publisher('/step_control', Int32, queue_size=1)
        self.sub = rospy.Subscriber('/step_control', Int32, self.find_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.step = 0

    def find_callback(self, msg):
        if msg.data == 20:
            self.step = 20
            twist = Twist()
            twist.linear.y = 0.5  # 사용하려는 축을 y에서 x로 변경
            twist.angular.z = 0.0  # z 축 회전은 없음
            self.pub.publish(twist)
            # self.step = 30

    def stop_robot(self, event):
        self.pub.publish(Twist())  # 로봇의 모든 움직임을 정지
        if self.step==30:
            self.step_pub.publish(30)  # 다음 단계로 상태 업데이트

    def run(self):
        rospy.spin()  # ROS 이벤트 루프 시작


if __name__ == '__main__':
    rospy.init_node('find_front')
    controller = RobotController()
    controller.run()