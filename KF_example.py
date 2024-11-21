# Install necessary library in Colab
!pip install filterpy

# Import libraries
import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter

# Time parameters
dt = 0.1  # Time step
time = np.arange(0, 10, dt)  # Total time

# True position and velocity
true_velocity = 1.0  # Constant velocity (m/s)
true_position = true_velocity * time

# Generate noisy measurements
np.random.seed(42)
measurement_noise = np.random.normal(0, 1, len(time))  # Gaussian noise
measurements = true_position + measurement_noise

# Initialize Kalman filter
kf = KalmanFilter(dim_x=2, dim_z=1)
kf.x = np.array([0, 0])  # Initial state: [position, velocity]
kf.F = np.array([[1, dt], [0, 1]])  # State transition matrix
kf.H = np.array([[1, 0]])  # Measurement matrix
kf.P = np.eye(2) * 500  # Covariance matrix (large initial uncertainty)
kf.R = 1  # Measurement noise covariance
kf.Q = np.array([[0.1, 0], [0, 0.1]])  # Process noise covariance

# Allocate space for results
estimated_positions = []
estimated_velocities = []

# Run the Kalman filter
for z in measurements:
    kf.predict()  # Prediction step
    kf.update(z)  # Update step with measurement
    estimated_positions.append(kf.x[0])
    estimated_velocities.append(kf.x[1])

# Plot results
plt.figure(figsize=(10, 6))

# Position
plt.subplot(2, 1, 1)
plt.plot(time, true_position, label="True Position")
plt.plot(time, measurements, label="Measurements", linestyle='dotted')
plt.plot(time, estimated_positions, label="Kalman Estimated Position")
plt.legend()
plt.title("Position Estimation")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")

# Velocity
plt.subplot(2, 1, 2)
plt.plot(time, [true_velocity] * len(time), label="True Velocity")
plt.plot(time, estimated_velocities, label="Kalman Estimated Velocity")
plt.legend()
plt.title("Velocity Estimation")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")

plt.tight_layout()
plt.show()
