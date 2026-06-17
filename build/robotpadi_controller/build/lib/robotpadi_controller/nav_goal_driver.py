#!/usr/bin/env python3
"""
Simple Nav Goal Driver for robotpadi.

Subscribes to /goal_pose (set via RViz 2D Nav Goal tool),
tracks robot pose via /tf (odom -> base_link),
then steers and drives the robot to that position.

No obstacle avoidance, no costmap, just pure geometry.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class NavGoalDriver(Node):

    # Tuning parameters
    WHEEL_RADIUS = 0.295       # meters (estimated from URDF z-offset of wheel)
    WHEEL_SPEED = 3.0          # rad/s (wheel angular velocity when driving)
    GOAL_TOLERANCE = 0.3       # meters — stop when this close to goal
    HEADING_TOLERANCE = 0.15   # radians — steer when heading error > this
    MAX_STEER_ANGLE = 0.6      # radians (~34 deg) max steering angle
    KP_STEER = 1.5             # proportional gain for steering

    def __init__(self):
        super().__init__('nav_goal_driver')

        # TF listener to get robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers for controllers
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_controller/commands',
            10
        )

        # Subscribe to RViz 2D Nav Goal
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # State
        self.goal = None       # (x, y) target in odom frame
        self.driving = False

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Nav Goal Driver started. Use RViz "2D Nav Goal" to set a target.')

    def goal_callback(self, msg: PoseStamped):
        """Received a new goal from RViz."""
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.driving = True
        self.get_logger().info(f'New goal: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}')

    def get_robot_pose(self):
        """Get current robot pose (x, y, yaw) in odom frame via TF."""
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            # Convert quaternion to yaw
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def set_wheels(self, speed):
        """Command all 4 wheels. Positive = forward."""
        msg = Float64MultiArray()
        # Revolute 5 (br), 6 (fl), 7 (bl), 8 (fr)
        # Check URDF axis directions: br/fr use +x axis, fl/bl use -x axis
        # So positive velocity on br/fr = forward, negative on fl/bl = forward
        msg.data = [speed, -speed, -speed, speed]
        self.wheel_pub.publish(msg)

    def set_steering(self, angle):
        """Command all 4 steering joints to the same angle (simplified)."""
        msg = Float64MultiArray()
        # Revolute 1 (fr), 2 (br), 3 (fl), 4 (bl)
        msg.data = [angle, angle, angle, angle]
        self.steer_pub.publish(msg)

    def stop(self):
        self.set_wheels(0.0)
        self.set_steering(0.0)

    def control_loop(self):
        if not self.driving or self.goal is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        rx, ry, ryaw = pose
        gx, gy = self.goal

        # Distance to goal
        dx = gx - rx
        dy = gy - ry
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.GOAL_TOLERANCE:
            self.get_logger().info('Goal reached!')
            self.stop()
            self.driving = False
            self.goal = None
            return

        # Angle to goal
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - ryaw

        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        while heading_error < -math.pi:
            heading_error += 2.0 * math.pi

        # Compute steer angle proportionally, clamp to max
        steer = self.KP_STEER * heading_error
        steer = max(-self.MAX_STEER_ANGLE, min(self.MAX_STEER_ANGLE, steer))

        self.set_steering(steer)
        self.set_wheels(self.WHEEL_SPEED)


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
