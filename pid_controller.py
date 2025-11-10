#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class PIDControllerNode(Node):
    """
    Subscribes:  'estimated_state' -> Float32MultiArray [position, velocity]
    Publishes:   'control_effort'  -> Float32 (u)
    Params:
        setpoint (float)        : desired position
        kp (float), ki (float), kd (float)
        u_min (float), u_max (float)    : output clamps
        use_vel_ff (bool)       : add velocity feedforward term (+kv * v_des), off by default
        kv (float)              : feedforward gain if use_vel_ff = true
        reset_on_nan (bool)     : reset integrator on bad input
    """
    def __init__(self):
        super().__init__('pid_controller')

        # ---- Parameters (declare with defaults) ----
        self.declare_parameter('setpoint', 1.0)
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('u_min', -float('inf'))
        self.declare_parameter('u_max',  float('inf'))
        self.declare_parameter('use_vel_ff', False)
        self.declare_parameter('kv', 0.0)
        self.declare_parameter('reset_on_nan', True)

        # Cache params
        self.setpoint = float(self.get_parameter('setpoint').value)
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.u_min = float(self.get_parameter('u_min').value)
        self.u_max = float(self.get_parameter('u_max').value)
        self.use_vel_ff = bool(self.get_parameter('use_vel_ff').value)
        self.kv = float(self.get_parameter('kv').value)
        self.reset_on_nan = bool(self.get_parameter('reset_on_nan').value)

        # ---- I/O ----
        self.sub = self.create_subscription(
            Float32MultiArray, 'estimated_state', self.state_cb, 10
        )
        self.pub = self.create_publisher(Float32, 'control_effort', 10)

        # ---- PID internals ----
        self.prev_time = None
        self.prev_err = 0.0
        self.int_err = 0.0

        self.get_logger().info(
            f"PID ready: setpoint={self.setpoint}, Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, "
            f"u∈[{self.u_min},{self.u_max}]"
        )

    def clamp(self, value, lo, hi):
        if value < lo:
            return lo, True
        if value > hi:
            return hi, True
        return value, False

    def state_cb(self, msg: Float32MultiArray):
        # Expect [position, velocity]
        try:
            x = float(msg.data[0])
            v = float(msg.data[1]) if len(msg.data) > 1 else 0.0
        except Exception as e:
            if self.reset_on_nan:
                self.int_err = 0.0
                self.prev_err = 0.0
            self.get_logger().warn(f"Bad state message ({e}); skipping")
            return

        now = self.get_clock().now()
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            # Guard against clock glitches
            if dt <= 0.0:
                dt = 0.0

        # Error (position control)
        err = self.setpoint - x

        # PID terms
        p = self.kp * err

        if dt > 0.0:
            self.int_err += err * dt
        i = self.ki * self.int_err

        d = 0.0
        if dt > 0.0:
            d = self.kd * (err - self.prev_err) / dt

        # Optional velocity feedforward (e.g., if you drive to a moving setpoint, set kv and desired v)
        ff = 0.0
        if self.use_vel_ff:
            # Here we assume desired velocity ≈ 0; override by setting kv and publishing a separate desired v if needed.
            # You can repurpose 'v' as v_des from another topic if you change the subscription.
            ff = self.kv * 0.0

        u_raw = p + i + d + ff

        # Clamp & simple anti-windup (stop integrating when saturated and pushing further into saturation)
        u, saturated = self.clamp(u_raw, self.u_min, self.u_max)
        if saturated and dt > 0.0:
            # If saturation direction matches integrator direction, back out the last integration step
            sign_push = np_sign(self.ki * self.int_err)
            sign_sat = np_sign(u_raw - u)
            if sign_push == sign_sat:
                self.int_err -= err * dt  # undo last integrator increment

        # Publish
        out = Float32()
        out.data = float(u)
        self.pub.publish(out)

        # Housekeeping
        self.prev_time = now
        self.prev_err = err

        # Debug trace (light)
        self.get_logger().debug(
            f"x={x:.3f}, v={v:.3f}, e={err:.3f}, dt={dt:.3f}, u={u:.3f} (P={p:.3f}, I={i:.3f}, D={d:.3f})"
        )

def np_sign(x: float) -> int:
    if x > 0.0:
        return 1
    if x < 0.0:
        return -1
    return 0

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
