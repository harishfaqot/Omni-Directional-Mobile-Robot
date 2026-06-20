#!/usr/bin/env python3
"""
robot_state_monitor.py
-----------------------
Node "jembatan" untuk robotpadi: membaca /odom dan /joint_states, lalu
mempublikasikan ulang sebagai topic Float64 individual dengan nama yang
PASTI dan STABIL -- supaya gampang dipakai di PlotJuggler tanpa harus
menebak-nebak index array.

Topic yang dipublikasikan:

  Full robot (dari /odom -> twist):
    /robot_state/vx      (m/s)   kecepatan linear sumbu X
    /robot_state/vy      (m/s)   kecepatan linear sumbu Y
    /robot_state/omega   (rad/s) kecepatan sudut Z (yaw rate)

  Kecepatan motor penggerak (dari /joint_states -> velocity, Revolute 5-8):
    /motor_state/omega1  (rad/s)
    /motor_state/omega2  (rad/s)
    /motor_state/omega3  (rad/s)
    /motor_state/omega4  (rad/s)

  Sudut motor swerve (dari /joint_states -> position, Revolute 1-4):
    /swerve_state/theta1 (rad)
    /swerve_state/theta2 (rad)
    /swerve_state/theta3 (rad)
    /swerve_state/theta4 (rad)

Pemetaan join (sesuai urdf/robotpadi.xacro & config/controllers.yaml):
    Theta1 <-> Revolute 1   |  Omega1 <-> Revolute 5
    Theta2 <-> Revolute 2   |  Omega2 <-> Revolute 6
    Theta3 <-> Revolute 3   |  Omega3 <-> Revolute 7
    Theta4 <-> Revolute 4   |  Omega4 <-> Revolute 8

Cara pakai cepat (tanpa build package, cukup punya workspace sudah di-source):
    chmod +x robot_state_monitor.py
    ./robot_state_monitor.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

STEER_JOINTS = ["Revolute 1", "Revolute 2", "Revolute 3", "Revolute 4"]  # -> theta1..4
WHEEL_JOINTS = ["Revolute 5", "Revolute 6", "Revolute 7", "Revolute 8"]  # -> omega1..4


class RobotStateMonitor(Node):

    def __init__(self):
        super().__init__('robot_state_monitor')

        # --- Full robot: Vx, Vy, Omega ---
        self.pub_vx = self.create_publisher(Float64, '/robot_state/vx', 10)
        self.pub_vy = self.create_publisher(Float64, '/robot_state/vy', 10)
        self.pub_omega = self.create_publisher(Float64, '/robot_state/omega', 10)

        # --- Kecepatan motor penggerak: Omega1..4 ---
        self.pub_wheel = [
            self.create_publisher(Float64, f'/motor_state/omega{i + 1}', 10)
            for i in range(4)
        ]

        # --- Sudut motor swerve: Theta1..4 ---
        self.pub_steer = [
            self.create_publisher(Float64, f'/swerve_state/theta{i + 1}', 10)
            for i in range(4)
        ]

        self.create_subscription(Odometry, '/odom', self.odom_cb, 50)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 50)

        self.get_logger().info(
            'robot_state_monitor aktif -> /robot_state/*, /motor_state/*, /swerve_state/*'
        )

    def odom_cb(self, msg: Odometry):
        t = msg.twist.twist
        self.pub_vx.publish(Float64(data=t.linear.x))
        self.pub_vy.publish(Float64(data=t.linear.y))
        self.pub_omega.publish(Float64(data=t.angular.z))

    def joint_cb(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        for i, jname in enumerate(STEER_JOINTS):
            idx = name_to_idx.get(jname)
            if idx is not None and idx < len(msg.position):
                self.pub_steer[i].publish(Float64(data=msg.position[idx]))

        for i, jname in enumerate(WHEEL_JOINTS):
            idx = name_to_idx.get(jname)
            if idx is not None and idx < len(msg.velocity):
                self.pub_wheel[i].publish(Float64(data=msg.velocity[idx]))


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
