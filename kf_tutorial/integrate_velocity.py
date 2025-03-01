import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

class Estimator(Node):

    def __init__(self):
        super().__init__('kf_estimator')
        self.publisher_ = self.create_publisher(PoseStamped, 'PositionEstimate', 10) #We will publish to this topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/VelocityStamped',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # other variables initialization
        self.time_prev = 0.0
        self.x_est = -0.66
        self.flag = 1

    def listener_callback(self, msg):

        # store
        self.vx_measurement = msg.pose.position.y
        self.time_now = float(msg.header.stamp.sec)*1e9 + float(msg.header.stamp.nanosec) #fetching time in nanoseconds
        # self.dt = min(float(self.time_now - self.time_prev)/1e9, 0.02)
        if self.flag:
            self.dt = 0.01
            self.flag = 0
        else:
            self.dt = float(self.time_now - self.time_prev)/1e9
        self.x_est = self.x_est + self.vx_measurement*self.dt
        self.time_prev = self.time_now

        # publish estimated position
        pub_msg = PoseStamped()
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        pub_msg.pose.position.x = self.x_est # this is estimated from just integration
        pub_msg.pose.position.y = msg.pose.position.x # this is actual estimate from PX4
        self.publisher_.publish(pub_msg)
        # self.get_logger().info('I heard and publishing: "%f"' % msg.vx)
      

def main(args=None):
    rclpy.init(args=args)

    kf_estimator = Estimator()

    rclpy.spin(kf_estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kf_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()