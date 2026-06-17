#!/usr/bin/env python3
"""
Swerve Drive Nav Goal Node
Subscribes to /goal_pose (set via RViz 2D Nav Goal button) and drives
the robot to the goal using swerve kinematics.

Control strategy:
  1. ROTATE  – spin in place to face the goal
  2. DRIVE   – move forward toward goal (always facing it)
  3. ALIGN   – spin in place to match goal heading
  4. DONE    – stop

Odometry source : /odom  (nav_msgs/msg/Odometry)
Goal source     : /goal_pose (geometry_msgs/msg/PoseStamped)  ← RViz Nav2 Goal
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import builtin_interfaces.msg


# ─── Robot geometry (must match swerve_controller.py) ────────────────────────
HALF_TRACK   = 0.950 / 2
HALF_BASE    = 1.090 / 2
WHEEL_RADIUS = 0.2

# ─── Tuning ───────────────────────────────────────────────────────────────────
GOAL_TOLERANCE_M    = 0.20   # metres  – stop driving when closer than this
GOAL_TOLERANCE_RAD  = 0.05   # radians – stop rotating when closer than this

MAX_LINEAR_SPEED    = 1.0    # m/s
MAX_ANGULAR_SPEED   = 1.0    # rad/s
MIN_ANGULAR_SPEED   = 0.15   # rad/s – minimum to overcome friction

LINEAR_KP           = 0.6    # proportional gain for distance
ANGULAR_KP          = 1.2    # proportional gain for heading error

ROTATE_FIRST_THRESH = 0.3    # rad – if heading error > this, rotate first


# ─── Helpers ─────────────────────────────────────────────────────────────────

def quaternion_to_yaw(q) -> float:
    """Extract yaw (rad) from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_wrap(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def swerve_kinematics(vx: float, vy: float, omega: float):
    """
    Same kinematics as swerve_controller.py.
    Returns [(speed_rad_s, steering_angle_rad), ...] for [RR, FL, RL, FR].
    """
    Lx, Ly = HALF_TRACK, HALF_BASE
    corners = [
        ( Ly,  Lx),   # FL
        (-Ly,  Lx),   # RL
        ( Ly, -Lx),   # FR
        (-Ly, -Lx),   # RR
    ]
    results = []
    for (cx, cy) in corners:
        vwx = vx - omega * cy
        vwy = vy + omega * cx
        speed_ms    = math.hypot(vwx, vwy)
        speed_rad_s = speed_ms / WHEEL_RADIUS
        angle       = math.atan2(vwy, vwx) if speed_ms > 1e-6 else 0.0
        results.append((speed_rad_s, angle))
    return results


# ─── States ──────────────────────────────────────────────────────────────────
IDLE    = "IDLE"
ROTATE  = "ROTATE"    # spin to face goal
DRIVE   = "DRIVE"     # drive toward goal while facing it
ALIGN   = "ALIGN"     # spin to match goal heading
DONE    = "DONE"

DESIRED_PATH_POINTS = 50    # Number of points in desired trajectory

class SwerveNavGoal(Node):
    def __init__(self):
        super().__init__('swerve_nav_goal')

        # Publishers
        self.wheel_pub    = self.create_publisher(
            Float64MultiArray, '/wheel_controller/commands', 10)
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', 10)
        
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.path_pub        = self.create_publisher(Path,   '/robot_path',  10)
        self.path_msg        = Path()
        self.path_msg.header.frame_id = 'odom'

        self.desired_path_pub = self.create_publisher(Path, '/desired_path', 10)
        self.desired_path_msg = Path()
        self.desired_path_msg.header.frame_id = 'odom'
        self.desired_path_points = []

        # Subscribers
        self.create_subscription(Odometry,     '/odom',      self._odom_cb,      10)
        self.create_subscription(PoseStamped,  '/goal_pose', self._goal_cb,      10)

        # State
        self.x     = 0.0
        self.y     = 0.0
        self.yaw   = 0.0
        self.odom_received = False

        self.goal_x   = None
        self.goal_y   = None
        self.goal_yaw = None
        self.state    = IDLE

        self.create_timer(0.05, self._control_loop)   # 20 Hz
        self.get_logger().info("SwerveNavGoal ready – set a 2D Nav Goal in RViz")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation) + math.pi / 2.0  # adjust for URDF orientation
        self.odom_received = True

        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = pose.header.stamp
        self.path_pub.publish(self.path_msg)

    def _goal_cb(self, msg: PoseStamped):
        self.goal_x   = msg.pose.position.x
        self.goal_y   = msg.pose.position.y
        self.goal_yaw = quaternion_to_yaw(msg.pose.orientation)
        self.state    = ROTATE
        self.get_logger().info(
            f"New goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, "
            f"yaw={math.degrees(self.goal_yaw):.1f}°)"
        )

        self._generate_desired_trajectory()
        self._publish_desired_path()
        
        self.get_logger().info(
            f"New goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, "
            f"yaw={math.degrees(self.goal_yaw):.1f}°)"
        )
        self.get_logger().info(f"Desired trajectory generated with {len(self.desired_path_points)} points")

        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns     = 'goal'
        m.id     = 0
        m.type   = Marker.ARROW
        m.action = Marker.ADD
        m.pose   = msg.pose
        m.scale.x = 0.5
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color.r = 1.0
        m.color.g = 0.3
        m.color.b = 0.0
        m.color.a = 1.0
        self.goal_marker_pub.publish(m)

        # Clear old path on new goal
        self.path_msg.poses.clear()


    def _generate_desired_trajectory(self):
        """Generate a straight-line trajectory from current position to goal."""
        if self.goal_x is None or self.goal_y is None:
            return
        
        self.desired_path_points.clear()
        
        # Current position
        start_x, start_y = self.x, self.y
        
        # Generate interpolated points along the straight line
        for i in range(DESIRED_PATH_POINTS + 1):
            t = i / DESIRED_PATH_POINTS
            x = start_x + t * (self.goal_x - start_x)
            y = start_y + t * (self.goal_y - start_y)
            
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            self.desired_path_points.append(pose)
            
    def _publish_desired_path(self):
        """Publish the desired trajectory as a Path message."""
        if not self.desired_path_points:
            return
        
        self.desired_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.desired_path_msg.poses = self.desired_path_points.copy()
        self.desired_path_pub.publish(self.desired_path_msg)
        
    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self.odom_received or self.state == IDLE or self.state == DONE:
            return

        dx   = self.goal_x - self.x
        dy   = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        # Angle from robot to goal (world frame)
        angle_to_goal  = math.atan2(dy, dx)
        heading_error  = angle_wrap(-angle_to_goal + self.yaw)
        align_error    = angle_wrap(-self.goal_yaw  + self.yaw)

        # ── State machine ─────────────────────────────────────────────────────

        if self.state == ROTATE:
            if abs(heading_error) < GOAL_TOLERANCE_RAD:
                self.get_logger().info("Heading aligned → DRIVE")
                self.state = DRIVE
            else:
                omega = self._angular_cmd(heading_error)
                self._publish(0.0, 0.0, omega)

        elif self.state == DRIVE:
            if dist < GOAL_TOLERANCE_M:
                self.get_logger().info("Goal reached → ALIGN")
                self._publish(0.0, 0.0, 0.0)
                self.state = ALIGN
                return

            # Correct heading continuously while driving
            omega = 0.0
            if abs(heading_error) > ROTATE_FIRST_THRESH:
                # Too far off – rotate in place first
                omega = self._angular_cmd(heading_error)
                self._publish(0.0, 0.0, omega)
                return

            # Small heading correction + forward drive
            vx    = min(LINEAR_KP * dist, MAX_LINEAR_SPEED)
            omega = self._angular_cmd(heading_error) * 0.5   # gentler while driving
            self._publish(vx, 0.0, omega)

        elif self.state == ALIGN:
            if abs(align_error) < GOAL_TOLERANCE_RAD:
                self.get_logger().info("Final heading aligned → DONE")
                self._publish(0.0, 0.0, 0.0)
                self.state = DONE
            else:
                omega = self._angular_cmd(align_error)
                self._publish(0.0, 0.0, omega)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _angular_cmd(self, error: float) -> float:
        """P-controller for angular velocity with minimum speed."""
        raw = ANGULAR_KP * error
        raw = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, raw))
        # Apply minimum speed so the robot doesn't stall near zero
        if 0 < abs(raw) < MIN_ANGULAR_SPEED:
            raw = math.copysign(MIN_ANGULAR_SPEED, raw)
        return raw

    def _publish(self, vx: float, vy: float, omega: float):
        """Convert (vx, vy, omega) → wheel + steering commands and publish.

        NOTE: If your robot's forward axis is Y (common with Solidworks URDF exports),
        swap vx↔vy here. Check in RViz: the GREEN arrow on base_link = Y axis.
        If GREEN points forward on your physical robot, use the swap below.
        """
        # ── Axis remap ────────────────────────────────────────────────────────
        # Uncomment ONE of these blocks depending on your robot's forward axis:

        # CASE A: Robot forward = Y axis (green arrow in RViz points forward)
        robot_vx, robot_vy = vx, vy   # swap

        # CASE B: Robot forward = X axis (red arrow in RViz points forward)
        # robot_vx, robot_vy = vx, vy  # no swap

        wheels = swerve_kinematics(robot_vx, robot_vy, omega)

        speed_signs = [-1, -1, 1, 1]

        wheel_msg    = Float64MultiArray()
        steering_msg = Float64MultiArray()
        wheel_msg.data    = [speed_signs[i] * wheels[i][0] for i in range(4)]
        steering_msg.data = [wheels[i][1]                  for i in range(4)]
        # ─────────────────────────────────────────────────────────────────────

        self.wheel_pub.publish(wheel_msg)
        self.steering_pub.publish(steering_msg)


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = SwerveNavGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")


if __name__ == '__main__':
    main()