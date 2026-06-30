import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')
        self.subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            10)

    def imu_callback(self, msg):
        self.get_logger().info(f"Accel: {msg.linear_acceleration}, Gyro: {msg.angular_velocity}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
