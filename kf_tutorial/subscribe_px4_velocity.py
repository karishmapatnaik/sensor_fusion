import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped


class MinimalSubPub(Node):

    def __init__(self):
        super().__init__('minimal_subpub')
        self.publisher_ = self.create_publisher(PoseStamped, 'VelocityStamped', 10)
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/vehicle_odometry/out',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        pub_msg = PoseStamped()
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        pub_msg.pose.position.x = msg.x
        pub_msg.pose.position.y = msg.vx
        self.publisher_.publish(pub_msg)
        # self.get_logger().info('I heard and publishing: "%f"' % msg.vx)
      

def main(args=None):
    rclpy.init(args=args)

    minimal_subpub = MinimalSubPub()

    rclpy.spin(minimal_subpub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subpub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()