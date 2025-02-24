import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class OdomToTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel/filtered', 10)
        self.wz = 0.0   

    def odom_callback(self, msg):

        if msg.angular.z < 1.5 and msg.angular.z > 0.0:
            self.wz = 1.5
        elif msg.angular.z > -1.5 and msg.angular.z < 0.0:
            self.wz = -1.5
        else:
            self.wz = msg.angular.z

        t = Twist()
        t.linear.x = msg.linear.x
        t.linear.y = msg.linear.y
        t.linear.z = msg.linear.z
        t.angular.x = msg.angular.x
        t.angular.y = msg.angular.y
        t.angular.z = self.wz


        self.publisher.publish(t)
        # self.get_logger().info("Published transform from odom to base_link")


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfBroadcaster()
    try:
        rclpy.spin(node)
    except:
        return 
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
