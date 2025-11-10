#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import math
import time
import os

# We'll save a plot at shutdown
import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt

class DoubleIntegratorSim(Node):
    """
    Subscribes:
        - 'control_effort' (Float32): control input u

    Publishes:
        - 'measurement' (Float32): measured position (for Kalman node)
        - 'true_state' (Float32MultiArray): [x, v] ground truth for debugging

    Parameters:
        dt (float): integration time step (s)
        x0 (float), v0 (float): initial conditions
        meas_noise_std (float): additive Gaussian noise on measurement
        setpoint (float): for plotting reference line (keep in sync with PID's setpoint)
        sim_duration (float): seconds; if >0, auto-shutdown after this duration (plot is saved on shutdown)
        plot_path (str): where to save the plot PNG
    """
    def __init__(self):
        super().__init__('double_integrator_sim')

        # --- params ---
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('v0', 0.0)
        self.declare_parameter('meas_noise_std', 0.0)
        self.declare_parameter('setpoint', 0.0)
        self.declare_parameter('sim_duration', 0.0)  # 0 means run until Ctrl+C
        self.declare_parameter('plot_path', 'double_integrator_response.png')

        self.dt = float(self.get_parameter('dt').value)
        self.x = float(self.get_parameter('x0').value)
        self.v = float(self.get_parameter('v0').value)
        self.meas_noise_std = float(self.get_parameter('meas_noise_std').value)
        self.setpoint = float(self.get_parameter('setpoint').value)
        self.sim_duration = float(self.get_parameter('sim_duration').value)
        self.plot_path = str(self.get_parameter('plot_path').value)

        # --- I/O ---
        self.u = 0.0
        self.sub_u = self.create_subscription(Float32, 'control_effort', self.u_cb, 10)
        self.pub_meas = self.create_publisher(Float32, 'measurement', 10)
        self.pub_state = self.create_publisher(Float32MultiArray, 'true_state', 10)

        # --- timer ---
        self.timer = self.create_timer(self.dt, self.step)

        # --- logs & buffers for plotting ---
        self.t0_wall = time.time()
        self.t_vec = []
        self.x_vec = []
        self.v_vec = []
        self.u_vec = []
        self.sp_vec = []

        self.get_logger().info(
            f"Double integrator sim started (dt={self.dt}s), x0={self.x}, v0={self.v}, "
            f"meas_noise_std={self.meas_noise_std}."
        )

    def u_cb(self, msg: Float32):
        self.u = float(msg.data)

    def step(self):
        # Integrate dynamics with constant u over [t, t+dt]
        u = self.u
        dt = self.dt

        # simple Euler
        self.x = self.x + self.v * dt
        self.v = self.v + u * dt

        # Publish true state
        s = Float32MultiArray()
        s.data = [float(self.x), float(self.v)]
        self.pub_state.publish(s)

        # Publish measurement (position with optional noise)
        y = float(self.x)
        if self.meas_noise_std > 0.0:
            # lightweight noise (Box–Muller)
            import random
            r1 = max(1e-12, random.random())
            r2 = random.random()
            z = math.sqrt(-2.0 * math.log(r1)) * math.cos(2.0 * math.pi * r2)
            y += self.meas_noise_std * z

        m = Float32()
        m.data = y
        self.pub_meas.publish(m)

        # Log for plot
        t = time.time() - self.t0_wall
        self.t_vec.append(t)
        self.x_vec.append(self.x)
        self.v_vec.append(self.v)
        self.u_vec.append(u)
        self.sp_vec.append(self.setpoint)

        # Optional auto-stop
        if self.sim_duration > 0.0 and t >= self.sim_duration:
            self.get_logger().info("Simulation duration reached; shutting down…")
            rclpy.shutdown()

    def save_plot(self):
        if len(self.t_vec) < 2:
            self.get_logger().warn("Not enough samples to plot.")
            return
        try:
            fig1 = plt.figure(figsize=(8, 5))
            plt.plot(self.t_vec, self.x_vec, label='position x')
            plt.plot(self.t_vec, self.sp_vec, linestyle='--', label='setpoint')
            plt.xlabel('time (s)')
            plt.ylabel('position')
            plt.title('Double Integrator Position')
            plt.legend()
            plt.grid(True)
            fig1.tight_layout()

            fig2 = plt.figure(figsize=(8, 4))
            plt.plot(self.t_vec, self.u_vec, label='control u')
            plt.xlabel('time (s)')
            plt.ylabel('control effort')
            plt.title('Control Effort')
            plt.legend()
            plt.grid(True)
            fig2.tight_layout()

            # Save both by appending suffixes
            base, ext = os.path.splitext(self.plot_path)
            pos_path = f"{base}_pos{ext or '.png'}"
            u_path = f"{base}_u{ext or '.png'}"
            fig1.savefig(pos_path, dpi=150)
            fig2.savefig(u_path, dpi=150)
            plt.close(fig1)
            plt.close(fig2)

            self.get_logger().info(f"Saved plots: {pos_path} and {u_path}")
        except Exception as e:
            self.get_logger().error(f"Plot save failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DoubleIntegratorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save plot before exit
        try:
            node.save_plot()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
