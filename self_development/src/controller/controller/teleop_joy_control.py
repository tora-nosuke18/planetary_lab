#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

msg = """
Reading from the Dualshock4  and Publishing to Twist!
---------------------------
Press "PS" button and Change the following modes :

    Mode 1 : Simple traverse. Press "Cross" buttons

    Mode 2 : Intuitive traverse. Push "Left Stick"
    
△  / × : increase/decrease only linear speed by 10%
○  / □ : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_joy_control')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 1)

        # モードとスケールの初期化
        self.mode = 1  # モード1でスタート
        self.vel_scale = 0.0
        self.angular_scale = 0.0

    def joy_callback(self, joy_msg):
        twist = Twist()

        # モード切り替えボタンが押されたとき
        if joy_msg.buttons[10] == 1:
            self.mode = 2 if self.mode == 1 else 1
            time.sleep(0.3)
            print(f"Mode switched to: {self.mode}")

        # スケーリング調整
        if joy_msg.buttons[2] == 1:
            self.vel_scale += 0.1
            time.sleep(0.3)

        elif joy_msg.buttons[0] == 1:
            self.vel_scale -= 0.1
            time.sleep(0.3)

        if joy_msg.buttons[1] == 1:
            self.angular_scale += 0.1
            time.sleep(0.3)

        elif joy_msg.buttons[3] == 1:
            self.angular_scale -= 0.1
            time.sleep(0.3)

        # モードに応じた軸の割り当てとスケーリング
        if self.mode == 1:
            twist.linear.x = joy_msg.axes[7] * (1 + self.vel_scale)
            twist.angular.z = joy_msg.axes[6] * (1 + self.angular_scale)

        elif self.mode == 2:
            twist.linear.x = joy_msg.axes[1] * (1 + self.vel_scale)
            twist.angular.z = joy_msg.axes[0] * (1 + self.angular_scale)

        # Twistメッセージをパブリッシュ
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()    
    try:
        print(msg)
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
