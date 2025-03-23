import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuFilter(Node):
    def __init__(self):
        super().__init__('imu_filter')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Imu, '/imu/filtered', 10)

    def imu_callback(self, msg):

        imu = Imu()
        imu.header = msg.header
        imu.linear_acceleration.x = msg.linear_acceleration.x - 0.3715
        imu.linear_acceleration.y = msg.linear_acceleration.y - 0.856
        imu.linear_acceleration.z = msg.linear_acceleration.z - 0.346
        imu.angular_velocity = msg.angular_velocity
        imu.angular_velocity_covariance = msg.angular_velocity_covariance
        imu.linear_acceleration_covariance = msg.linear_acceleration_covariance
        imu.orientation = msg.orientation
        imu.orientation_covariance = msg.orientation_covariance
        self.publisher.publish(imu)
        # self.get_logger().info("Published transform from odom to base_link")


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilter()
    try:
        rclpy.spin(node)
    except:
        return 
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
