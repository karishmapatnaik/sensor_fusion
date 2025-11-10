import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
from filterpy.kalman import KalmanFilter

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # Subscriber for measurements
        self.subscription = self.create_subscription(
            Float32, 'measurement', self.measurement_callback, 10)

        # Publisher for estimated state
        self.state_publisher = self.create_publisher(Float32MultiArray, 'estimated_state', 10)

        # Kalman filter initialization
        dt = 0.01  # Time step
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        self.kf.x = np.array([0, 0])  # Initial state: [position, velocity]
        self.kf.F = np.array([[1, dt], [0, 1]])  # State transition matrix
        self.kf.H = np.array([[1, 0]])  # Measurement matrix
        self.kf.P = np.eye(2) * 500  # Covariance matrix (large initial uncertainty)
        self.kf.R = 1  # Measurement noise covariance
        self.kf.Q = np.array([[0.1, 0], [0, 0.1]])  # Process noise covariance

    def measurement_callback(self, msg):
        # Predict and update steps
        measurement = msg.data
        self.kf.predict()
        self.kf.update(measurement)

        # Prepare estimated state to publish
        estimated_state = Float32MultiArray()
        estimated_state.data = [self.kf.x[0], self.kf.x[1]]  # [position, velocity]

        # Publish the estimated state
        self.state_publisher.publish(estimated_state)
        self.get_logger().info(f"Published Estimated State: {estimated_state.data}")


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
