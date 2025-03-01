import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class Estimator(Node):
    def __init__(self):
        super().__init__('kf_estimator')
        self.publisher_ = self.create_publisher(PoseStamped, 'PositionEstimate', 10)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/VelocityStamped',
            self.listener_callback,
            10)
        
        # Kalman Filter State
        self.x_est = np.array([[-0.66], [0.0]])  # [position; velocity]
        self.P = np.eye(2)
        self.Q = np.array([[0.001, 0.0], [0.0, 0.001]])
        self.R = np.array([[0.24]])
        self.H = np.array([[0.0, 1.0]])
        self.I = np.eye(2)
        self.time_prev = None
        self.noise_std_dev = 0.5  # Standard deviation of Gaussian white noise

    def listener_callback(self, msg):
        vx_measurement = float(msg.pose.position.y) + np.random.normal(0, self.noise_std_dev)
        time_now = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        dt = 0.01 if self.time_prev is None else max(time_now - self.time_prev, 0.001)
        self.time_prev = time_now

        # Kalman Prediction
        A = np.array([[1.0, dt], [0.0, 1.0]])
        self.x_est = A @ self.x_est
        self.P = A @ self.P @ A.T + self.Q

        # Kalman Update
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.x_est += K @ (vx_measurement - (self.H @ self.x_est))
        self.P = (self.I - K @ self.H) @ self.P

        # Publish estimated position
        pub_msg = PoseStamped()
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        pub_msg.pose.position.x = float(self.x_est[0, 0])
        pub_msg.pose.position.y = msg.pose.position.x
        self.publisher_.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    kf_estimator = Estimator()
    rclpy.spin(kf_estimator)
    kf_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
