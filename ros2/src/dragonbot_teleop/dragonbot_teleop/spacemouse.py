#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import pyspacemouse


def apply_deadzone(x: float, deadzone_size: float = 0.05) -> float:
    """Deadzone + rescale to keep full range outside deadzone."""
    ax = abs(x)
    if ax <= deadzone_size:
        return 0.0
    return np.sign(x) * (ax - deadzone_size) / (1.0 - deadzone_size)

class LowPass:
    """
    Simple 1-pole IIR low-pass:
        y <- alpha*x + (1-alpha)*y
    Note: This is "smoothing". Typical alpha in [0.7 .. 0.98].
    """
    def __init__(self, alpha: float = 0.9):
        self.alpha = float(alpha)
        self.y = 0.0

    def reset(self):
        self.y = 0.0

    def update(self, x: float) -> float:
        self.y = self.alpha * float(x) + (1.0 - self.alpha) * self.y
        return self.y



class SpaceMouseTwistPublisher(Node):
    def __init__(self):
        super().__init__("spacemouse_twist_publisher")

        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("deadzone", 0.05) 
        self.declare_parameter("alpha", 1.0)

        self.declare_parameter("scale_x", 1.0)
        self.declare_parameter("scale_y", 1.0)
        self.declare_parameter("scale_theta", 1.0)

        self.rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.deadzone = self.get_parameter("deadzone").get_parameter_value().double_value
        alpha = self.get_parameter("alpha").get_parameter_value().double_value

        self.scale_x = self.get_parameter("scale_x").get_parameter_value().double_value
        self.scale_y = self.get_parameter("scale_y").get_parameter_value().double_value
        self.scale_theta = self.get_parameter("scale_theta").get_parameter_value().double_value

        self.pub = self.create_publisher(Twist, "spacemouse/cmd_vel", 10)

        self.filters = {
            "x": LowPass(alpha),
            "y": LowPass(alpha),
            "rz": LowPass(alpha),
        }

        if not pyspacemouse.open():
            raise RuntimeError("Failed to open SpaceMouse (pyspacemouse.open() returned False)")
        self.get_logger().info("SpaceMouse ready. Publishing Twist...")

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self._on_timer)


    def _filtered(self, raw: float, key: str) -> float:
        dz = apply_deadzone(raw, self.deadzone)
        return float(self.filters[key].update(dz))

    def _on_timer(self):
        state = pyspacemouse.read()
        if state is None:
            self.get_logger().warn("None state read from SpaceMouse")
            return

        # SpaceMouse axes (pyspacemouse uses: x,y,z, roll,pitch,yaw)
        x  = self._filtered(state.x, "x") * self.scale_x
        y  = self._filtered(state.y, "y") * self.scale_y
        rz = self._filtered(state.yaw, "rz") * self.scale_theta  # theta from yaw

        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = rz
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SpaceMouseTwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()