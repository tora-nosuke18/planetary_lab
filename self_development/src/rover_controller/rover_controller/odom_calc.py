import math
from math import sin, cos, pi

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdometryCalc(Node):
    def __init__(self):
        super().__init__('odom_calc')

        self.r = 0.075  # 車輪半径
        self.TREAD = 0.2  # トレッド幅

        self.x = 0.0  # x座標
        self.y = 0.0  # y座標
        self.th = 0.0  # 姿勢

        self.vx = 0.0  # x方向速度
        self.vy = 0.0  # y方向速度
        self.vth = 0.0  # 角速度

        self.prev_left_enc = 0.0  # 前回の左エンコーダ値
        self.prev_right_enc = 0.0  # 前回の右エンコーダ値

        self.prev_time = self.get_clock().now()  # 前回の時間
        
        self.current_left_enc = 0.0  # 現在の左エンコーダ値
        self.current_right_enc = 0.0  # 現在の右エンコーダ値

        self.odom_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float64MultiArray, 'wheel_rps', self.enc_cb, 10) #/C620/actual_rad
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

    def enc_cb(self, msg):
        # エンコーダ値を取得
        left_enc = msg.data[0]  # 左エンコーダ値
        right_enc = msg.data[1]  # 右エンコーダ値

        # 経過時間を計算
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # ナノ秒を秒に変換

        # 角速度から速度を計算
        self.vx = self.r * (right_enc + left_enc) / 2
        self.vy = 0.0
        self.vth = self.r * (right_enc - left_enc) / (dt * self.TREAD)
        
        # 速度からオドメトリを計算
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y  
        self.th += delta_th

        # クオータニオンを計算
        odom_quat = quaternion_from_euler(0, 0, self.th)

        # tf変換を送信
        # odom_trans = TransformStamped()
        # odom_trans.header.stamp = current_time.to_msg()
        # odom_trans.header.frame_id = "odom"
        # odom_trans.child_frame_id = "base_link"

        # odom_trans.transform.translation.x = self.x
        # odom_trans.transform.translation.y = self.y
        # odom_trans.transform.translation.z = 0.0
        # odom_trans.transform.rotation.x = odom_quat[0]
        # odom_trans.transform.rotation.y = odom_quat[1]
        # odom_trans.transform.rotation.z = odom_quat[2]
        # odom_trans.transform.rotation.w = odom_quat[3]

        # self.odom_broadcaster.sendTransform(odom_trans)

        # オドメトリメッセージをROS 2で配信
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        self.get_logger().info(f"X {self.x:.3f},Y {self.y:.3f}")

        # オドメトリを配信
        self.odom_pub.publish(odom)
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    odom_calc = OdometryCalc()
    try:
        rclpy.spin(odom_calc)
    except:
        return
    odom_calc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
