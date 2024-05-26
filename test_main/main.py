import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class CmdVelMultiplexer:
    def __init__(self):
        rospy.init_node('cmd_vel_multiplexer', anonymous=True)
        self.current_step = None
        
        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        self.step_sub = rospy.Subscriber('/step_control', Int32, self.step_callback)
        self.follow_sub = rospy.Subscriber('/follow/cmd_vel', Twist, self.follow_cmd_callback)
        self.find_sub = rospy.Subscriber('/find/cmd_vel', Twist, self.find_cmd_callback)
        
        # Store the last commands from follow and find topics
        self.last_follow_cmd = Twist()
        self.last_find_cmd = Twist()

    def step_callback(self, msg):
        self.current_step = msg.data
        if self.current_step == 0:
            # Stop the robot if step is 0
            self.cmd_pub.publish(Twist())
        elif self.current_step == 10:
            # Use the last command from /follow/cmd_vel
            self.cmd_pub.publish(self.last_follow_cmd)
        elif self.current_step == 20:
            # Use the last command from /find/cmd_vel
            self.cmd_pub.publish(self.last_find_cmd)

    def follow_cmd_callback(self, msg):
        # Update the last command received from /follow/cmd_vel
        self.last_follow_cmd = msg
        # If the current step is 10, use this command
        if self.current_step == 10:
            self.cmd_pub.publish(msg)

    def find_cmd_callback(self, msg):
        # Update the last command received from /find/cmd_vel
        self.last_find_cmd = msg
        # If the current step is 20, use this command
        if self.current_step == 20:
            self.cmd_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    mux = CmdVelMultiplexer()
    mux.run()
