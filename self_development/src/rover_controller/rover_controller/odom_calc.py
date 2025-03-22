#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class DiffDriveOdom(Node):
    def __init__(self):
        super().__init__('diff_drive_odom')

        # パラメータの宣言
        self.declare_parameter('wheel_base', 0.2)  # 車輪間距離[m]
        self.declare_parameter('wheel_radius', 0.05)  # 車輪半径[m]
        self.declare_parameter('publish_tf', False)  # TFのブロードキャスト

        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'wheel_rps',
            self.wheel_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def wheel_callback(self, msg):
        try:
            left_rps = msg.data[0]
            right_rps = msg.data[1]
        except IndexError:
            self.get_logger().warn("wheel_rpsデータが不完全です")
            return

        # 現在時刻と経過時間
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # 直線速度・角速度を計算
        v_left = left_rps * self.wheel_radius
        v_right = right_rps * self.wheel_radius

        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        # 位置・姿勢の更新
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # オドメトリメッセージの作成
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # 四元数変換
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)
        self.get_logger().info('Current Position X: %f, Y: %f' % (self.x, self.y))

        # TFのブロードキャスト
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
