#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
import threading
from rclpy.executors import MultiThreadedExecutor
import sys

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publishers
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_controller/commands',
            10
        )
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )
        
        # Current state
        self.speed = 0.0  # m/s
        self.steering_angle = 0.0  # rad
        self.max_speed = 1.0  # m/s
        self.max_steering = 3  # rad (about 30 degrees)
        
        # Start the publisher timer
        self.timer = self.create_timer(0.05, self.publish_commands)
        
        # Log that node is ready
        self.get_logger().info("Robot Controller Node initialized")
        
    def publish_commands(self):
        # Calculate wheel speeds (all wheels same speed)
        wheel_speed = self.speed * 10  # Convert m/s to rad/s
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [-wheel_speed, wheel_speed, wheel_speed, -wheel_speed]
        
        # Front wheels
        left_front = self.steering_angle
        right_front = self.steering_angle
        # Rear wheels
        left_rear = self.steering_angle
        right_rear = self.steering_angle
        
        steering_msg = Float64MultiArray()
        steering_msg.data = [left_front, right_front, left_rear, right_rear]
        
        # Publish commands
        self.wheel_pub.publish(wheel_msg)
        self.steering_pub.publish(steering_msg)
        
        # Optional: Log for debugging (uncomment if needed)
        # self.get_logger().debug(f"Published: speed={self.speed:.2f}, steering={self.steering_angle:.2f}")

class RobotGUI:
    def __init__(self, controller_node):
        self.node = controller_node
        self.root = tk.Tk()
        self.root.title("Robot Controller")
        self.root.geometry("400x450")
        self.root.resizable(False, False)
        
        # Bind keyboard events
        self.root.bind('<KeyPress>', self.key_press)
        self.root.bind('<KeyRelease>', self.key_release)
        
        # Create GUI elements
        self.create_widgets()
        
        # Focus on window for keyboard input
        self.root.focus_set()
        
        # Flag to control ROS spinning
        self.running = True
        
    def create_widgets(self):
        # Title
        title_label = tk.Label(
            self.root,
            text="Robot Control Panel",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        # Keyboard instructions
        instr_frame = tk.Frame(self.root)
        instr_frame.pack(pady=5)
        
        tk.Label(
            instr_frame,
            text="Keyboard: W/S=Speed, A/D=Steering, Space=Stop, Q=Quit",
            font=("Arial", 10)
        ).pack()
        
        # Speed Slider
        speed_frame = tk.Frame(self.root)
        speed_frame.pack(pady=15)
        
        tk.Label(speed_frame, text="Speed (m/s):", font=("Arial", 10, "bold")).pack()
        self.speed_var = tk.DoubleVar(value=0.0)
        self.speed_slider = tk.Scale(
            speed_frame,
            from_=-2.0,
            to=2.0,
            resolution=0.1,
            orient=tk.HORIZONTAL,
            variable=self.speed_var,
            length=300,
            command=self.update_speed
        )
        self.speed_slider.pack()
        
        # Speed value display
        self.speed_label = tk.Label(speed_frame, text="0.0 m/s", font=("Arial", 10))
        self.speed_label.pack()
        
        # Steering Slider
        steer_frame = tk.Frame(self.root)
        steer_frame.pack(pady=15)
        
        tk.Label(steer_frame, text="Steering (rad):", font=("Arial", 10, "bold")).pack()
        self.steer_var = tk.DoubleVar(value=0.0)
        self.steer_slider = tk.Scale(
            steer_frame,
            from_=-3.0,
            to=3.0,
            resolution=0.05,
            orient=tk.HORIZONTAL,
            variable=self.steer_var,
            length=300,
            command=self.update_steering
        )
        self.steer_slider.pack()
        
        # Steering value display
        self.steer_label = tk.Label(steer_frame, text="0.00 rad", font=("Arial", 10))
        self.steer_label.pack()
        
        # Status display
        status_frame = tk.Frame(self.root)
        status_frame.pack(pady=10)
        
        self.status_label = tk.Label(
            status_frame,
            text="Status: Stopped",
            font=("Arial", 12, "bold")
        )
        self.status_label.pack()
        
        # Control buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)
        
        tk.Button(
            button_frame,
            text="STOP",
            command=self.stop_robot,
            bg="red",
            fg="white",
            font=("Arial", 12, "bold"),
            width=15,
            height=2
        ).pack(pady=5)
        
        tk.Button(
            button_frame,
            text="QUIT",
            command=self.quit_application,
            bg="gray",
            font=("Arial", 10),
            width=10
        ).pack()
        
    def update_speed(self, event=None):
        speed = self.speed_var.get()
        self.node.speed = speed
        self.speed_label.config(text=f"{speed:.1f} m/s")
        self.update_status()
        
    def update_steering(self, event=None):
        steering = self.steer_var.get()
        self.node.steering_angle = steering
        self.steer_label.config(text=f"{steering:.2f} rad")
        self.update_status()
        
    def update_status(self):
        speed = self.node.speed
        steering = self.node.steering_angle
        
        if abs(speed) < 0.01 and abs(steering) < 0.01:
            self.status_label.config(text="Status: Stopped", fg="red")
        elif speed > 0.01:
            direction = "Forward"
            if steering > 0.05:
                direction += " ↶ Left"
            elif steering < -0.05:
                direction += " ↷ Right"
            self.status_label.config(text=f"Status: {direction}", fg="green")
        elif speed < -0.01:
            direction = "Backward"
            if steering > 0.05:
                direction += " ↶ Left"
            elif steering < -0.05:
                direction += " ↷ Right"
            self.status_label.config(text=f"Status: {direction}", fg="blue")
        else:
            self.status_label.config(text="Status: Stopped", fg="red")
        
    def stop_robot(self):
        self.speed_var.set(0.0)
        self.steer_var.set(0.0)
        self.node.speed = 0.0
        self.node.steering_angle = 0.0
        self.speed_label.config(text="0.0 m/s")
        self.steer_label.config(text="0.00 rad")
        self.status_label.config(text="Status: Stopped", fg="red")
        
    def key_press(self, event):
        key = event.keysym.lower()
        
        if key == 'w':
            current = self.speed_var.get()
            new_speed = min(current + 0.3, 2.0)
            self.speed_var.set(new_speed)
            self.update_speed()
        elif key == 's':
            current = self.speed_var.get()
            new_speed = max(current - 0.3, -2.0)
            self.speed_var.set(new_speed)
            self.update_speed()
        elif key == 'a':
            current = self.steer_var.get()
            new_steer = min(current + 0.1, 3.0)
            self.steer_var.set(new_steer)
            self.update_steering()
        elif key == 'd':
            current = self.steer_var.get()
            new_steer = max(current - 0.1, -3.0)
            self.steer_var.set(new_steer)
            self.update_steering()
        elif key == 'space':
            self.stop_robot()
        elif key == 'q':
            self.quit_application()
            
    def key_release(self, event):
        # Optional: Add key release behavior if desired
        pass
            
    def quit_application(self):
        self.running = False
        self.stop_robot()
        self.root.quit()
        self.root.destroy()
        
    def run(self):
        self.root.mainloop()

def spin_ros(node, executor):
    """Spin ROS2 in a separate thread"""
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)
    except Exception as e:
        print(f"ROS spinning error: {e}")
    finally:
        print("ROS spinning stopped")

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create node
    controller_node = RobotController()
    
    # Create GUI
    gui = RobotGUI(controller_node)
    
    # Create executor and add node
    executor = MultiThreadedExecutor()
    executor.add_node(controller_node)
    
    # Start ROS spinning in a separate thread
    ros_thread = threading.Thread(
        target=spin_ros, 
        args=(controller_node, executor),
        daemon=True
    )
    ros_thread.start()
    
    print("Robot Controller started. Use keyboard or GUI to control.")
    print("W: Increase speed, S: Decrease speed")
    print("A: Turn left, D: Turn right")
    print("Space: Stop, Q: Quit")
    
    # Run the GUI in the main thread
    try:
        gui.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Cleanup
        print("Cleaning up...")
        executor.shutdown()
        controller_node.destroy_node()
        rclpy.shutdown()
        print("Goodbye!")

if __name__ == '__main__':
    main()