#!/usr/bin/env python3
"""
Swerve Drive Robot Controller
Proper swerve drive kinematics: each wheel has independent speed + steering angle.

Robot frame (top-down view):
        FRONT
   FL -------- FR
   |            |
   |    (Lx)   |
   |            |
   RL -------- RR
        REAR

Lx = half track width (left-right)
Ly = half wheelbase (front-rear)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
import tkinter.font as tkfont
import tkinter.ttk as ttk
import threading
from rclpy.executors import MultiThreadedExecutor


# ─── Robot geometry (metres) ──────────────────────────────────────────────────
HALF_TRACK   = 0.954/2   # Lx – half the distance between left and right wheels
HALF_BASE    = 1.090/2   # Ly – half the distance between front and rear axles

# ─── Limits ───────────────────────────────────────────────────────────────────
MAX_SPEED    = 2.0   # m/s (translation magnitude)
MAX_OMEGA    = 3.0   # rad/s (rotation)
WHEEL_RADIUS = 0.2  # metres – used to convert m/s → rad/s for the motor


def swerve_kinematics(vx: float, vy: float, omega: float):
    """
    Standard swerve-drive inverse kinematics.

    Parameters
    ----------
    vx    : forward  velocity  (m/s, robot frame, +x = forward)
    vy    : sideways velocity  (m/s, robot frame, +y = left)
    omega : yaw rate           (rad/s, +CCW when viewed from above)

    Returns
    -------
    List of (wheel_speed_rad_s, steering_angle_rad) for
        [FL, FR, RL, RR]   ← same order as your ROS topics
    """
    Lx, Ly = HALF_TRACK, HALF_BASE

    # Corner velocity contributions from rotation
    # Each corner is at (±Ly, ±Lx) in robot frame
    corners = [
        (-Ly, -Lx),   # RR
        ( Ly,  Lx),   # FL
        (-Ly,  Lx),   # RL
        ( Ly, -Lx),   # FR
        
    ]

    results = []
    for (cx, cy) in corners:
        # Wheel velocity vector = translation + rotation contribution
        vwx = vx - omega * cy   # forward component
        vwy = vy + omega * cx   # lateral component

        speed_ms     = math.hypot(vwx, vwy)
        speed_rad_s  = speed_ms / WHEEL_RADIUS

        # atan2 gives angle in [-π, π]; 0 = forward, +π/2 = left
        angle = math.atan2(vwy, vwx) if speed_ms > 1e-6 else 0.0

        results.append((speed_rad_s, angle))

    return results   # [(spd, ang), ...] for FL, FR, RL, RR


# ─── ROS2 Node ────────────────────────────────────────────────────────────────

class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')

        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_controller/commands', 10)
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', 10)

        # Robot velocity commands (robot frame)
        self.vx    = 0.0   # forward  m/s
        self.vy    = 0.0   # sideways m/s  (+left)
        self.omega = 0.0   # yaw rate rad/s (+CCW)

        self.create_timer(0.05, self._publish)   # 20 Hz
        self.get_logger().info("Swerve Controller ready")

    def _publish(self):
        wheels = swerve_kinematics(self.vx, self.vy, self.omega)

        # Wheel speed signs: FL and RR are mirrored on the frame
        # Adjust for your physical motor orientations here if needed.
        # Convention used: positive = forward rotation for each wheel.
        speed_signs = [-1, 1, 1, -1]   # matches your original sign pattern

        wheel_msg    = Float64MultiArray()
        steering_msg = Float64MultiArray()

        wheel_msg.data    = [speed_signs[i] * wheels[i][0] for i in range(4)]
        steering_msg.data = [wheels[i][1]                  for i in range(4)]

        self.wheel_pub.publish(wheel_msg)
        self.steering_pub.publish(steering_msg)


# ─── GUI ──────────────────────────────────────────────────────────────────────

DARK   = "#f0f4f8"   # window background (light)
MID    = "#dce6f0"   # panel / canvas background
ACCENT = "#3a7bd5"   # slider track / button fill
CYAN   = "#1a5fb4"   # arrow, headings, key hints
GREEN  = "#1e7e34"   # moving status
RED    = "#c0392b"   # stopped / stop button
WHITE  = "#1c1c2e"   # primary text (dark on light)
GRAY   = "#5a6a7a"   # secondary text


class WheelDiagram(tk.Canvas):
    """Small canvas that draws the 4-wheel state diagram."""

    W, H = 220, 200

    def __init__(self, parent, **kw):
        super().__init__(parent, width=self.W, height=self.H,
                         bg=MID, highlightthickness=0, **kw)
        self._wheels = [(0.0, 0.0)] * 4   # (speed_rad_s, angle_rad)
        self._labels  = ["FL", "FR", "RL", "RR"]
        self._cx      = [55, 165, 55, 165]
        self._cy      = [50,  50, 150, 150]
        self._draw()

    def update_wheels(self, wheels):
        self._wheels = wheels
        self._draw()

    def _draw(self):
        self.delete("all")
        # Robot body outline
        self.create_rectangle(35, 30, 185, 170,
                               outline=GRAY, width=1, dash=(4, 3))
        # Direction arrow (forward = up)
        self.create_line(110, 100, 110, 60,
                          fill=CYAN, width=2, arrow=tk.LAST)
        self.create_text(110, 50, text="FRONT",
                          fill=CYAN, font=("Courier", 7))

        max_spd = max((abs(w[0]) for w in self._wheels), default=1) or 1

        for i, (spd, ang) in enumerate(self._wheels):
            cx, cy = self._cx[i], self._cy[i]

            # Wheel rect rotated by ang
            # We draw a line to represent wheel direction
            length = 18
            cos_a, sin_a = math.cos(ang), math.sin(ang)
            # ang=0 means forward (up in canvas)
            # canvas: x right, y down → forward = -y
            dx = length * sin_a     # lateral in canvas
            dy = -length * cos_a    # forward in canvas (inverted y)

            intensity = min(int(abs(spd) / max_spd * 180 + 60), 240)
            color = f"#{0:02x}{intensity//2:02x}{intensity:02x}" if spd >= 0 \
                    else f"#{intensity:02x}{0:02x}{0:02x}"

            self.create_line(cx - dx, cy - dy, cx + dx, cy + dy,
                              fill=color, width=4, capstyle=tk.ROUND)
            # Speed text
            self.create_text(cx, cy + 18,
                              text=f"{spd:.1f}r/s",
                              fill=GRAY, font=("Courier", 7))
            self.create_text(cx, cy - 22,
                              text=self._labels[i],
                              fill=WHITE, font=("Courier", 8, "bold"))


class SwerveGUI:
    def __init__(self, node: SwerveController):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Swerve Drive Controller")
        self.root.geometry("480x720")
        self.root.resizable(False, False)
        self.root.configure(bg=DARK)

        self._build()
        self.root.bind('<KeyPress>',   self._key_press)
        self.root.bind('<KeyRelease>', self._key_release)
        self.root.focus_set()

        # Held keys for smooth driving
        self._held = set()
        self._key_timer()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _build(self):
        bold = ("Courier", 10, "bold")
        norm = ("Courier", 9)

        # ── Title ──
        tk.Label(self.root, text="SWERVE DRIVE", bg=DARK, fg=CYAN,
                 font=("Courier", 18, "bold")).pack(pady=(14, 2))
        tk.Label(self.root, text="independent wheel kinematics", bg=DARK, fg=GRAY,
                 font=("Courier", 8)).pack()

        # ── Wheel diagram ──
        self.diagram = WheelDiagram(self.root)
        self.diagram.pack(pady=10)

        # ── Sliders ──
        sliders_frame = tk.Frame(self.root, bg=DARK)
        sliders_frame.pack(padx=20, fill=tk.X)

        self.vx_var    = tk.DoubleVar(value=0.0)
        self.vy_var    = tk.DoubleVar(value=0.0)
        self.omega_var = tk.DoubleVar(value=0.0)

        def make_slider(parent, label, var, lo, hi, unit):
            f = tk.Frame(parent, bg=DARK)
            f.pack(fill=tk.X, pady=6)

            tk.Label(f, text=label, bg=DARK, fg=WHITE, font=bold,
                     width=12, anchor="w").pack(side=tk.LEFT)

            # ── Custom canvas slider ──────────────────────────────
            TRACK_W, TRACK_H, THUMB_R = 220, 6, 9
            CW, CH = TRACK_W + THUMB_R * 2 + 4, THUMB_R * 2 + 10
            c = tk.Canvas(f, width=CW, height=CH, bg=DARK,
                          highlightthickness=0, cursor="hand2")
            c.pack(side=tk.LEFT, padx=4)

            tx0 = THUMB_R + 2          # track left x
            tx1 = tx0 + TRACK_W        # track right x
            ty  = CH // 2              # track y centre

            def _val_to_x(v):
                return tx0 + (v - lo) / (hi - lo) * TRACK_W

            def _x_to_val(x):
                raw = lo + (x - tx0) / TRACK_W * (hi - lo)
                # snap to resolution 0.05
                snapped = round(raw / 0.05) * 0.05
                return max(lo, min(hi, snapped))

            def _redraw(*_):
                v  = var.get()
                cx = _val_to_x(v)
                c.delete("all")
                # full trough (light grey)
                c.create_rectangle(tx0, ty - TRACK_H//2,
                                   tx1, ty + TRACK_H//2 + 1,
                                   fill="#c8d8e8", outline="", tags="trough")
                # filled portion (accent blue)
                mid = _val_to_x(0)
                x0f, x1f = (min(mid, cx), max(mid, cx))
                if x1f > x0f:
                    c.create_rectangle(x0f, ty - TRACK_H//2,
                                       x1f, ty + TRACK_H//2 + 1,
                                       fill=ACCENT, outline="", tags="fill")
                # thumb circle
                c.create_oval(cx - THUMB_R, ty - THUMB_R,
                              cx + THUMB_R, ty + THUMB_R,
                              fill=ACCENT, outline="#ffffff", width=2, tags="thumb")

            var.trace_add("write", _redraw)
            _redraw()

            def _click(e):
                var.set(_x_to_val(e.x))
                self._on_slider()
            def _drag(e):
                var.set(_x_to_val(e.x))
                self._on_slider()

            c.bind("<Button-1>",        _click)
            c.bind("<B1-Motion>",       _drag)
            # mouse-wheel nudge
            def _wheel(e):
                delta = 0.05 if (e.delta > 0 or e.num == 4) else -0.05
                var.set(max(lo, min(hi, round((var.get() + delta) / 0.05) * 0.05)))
                self._on_slider()
            c.bind("<MouseWheel>", _wheel)
            c.bind("<Button-4>",   _wheel)
            c.bind("<Button-5>",   _wheel)
            # ─────────────────────────────────────────────────────

            lbl = tk.Label(f, text=f"0.00 {unit}", bg=DARK, fg=CYAN,
                           font=norm, width=9, anchor="e")
            lbl.pack(side=tk.LEFT, padx=6)
            return lbl

        self._vx_lbl    = make_slider(sliders_frame, "Forward  vx",
                                      self.vx_var,    -MAX_SPEED, MAX_SPEED,  "m/s")
        self._vy_lbl    = make_slider(sliders_frame, "Sideways vy",
                                      self.vy_var,    -MAX_SPEED, MAX_SPEED,  "m/s")
        self._omega_lbl = make_slider(sliders_frame, "Rotation ω",
                                      self.omega_var, -MAX_OMEGA, MAX_OMEGA, "r/s")

        # ── Keyboard hint ──
        hint = tk.Frame(self.root, bg=MID, bd=0)
        hint.pack(padx=20, pady=8, fill=tk.X)
        keys = [
            ("W/S", "Forward / Back"),
            ("A/D", "Strafe Left / Right"),
            ("Q/E", "Rotate CCW / CW"),
            ("Space", "Stop all"),
        ]
        for k, v in keys:
            row = tk.Frame(hint, bg=MID)
            row.pack(anchor="w", padx=8, pady=1)
            tk.Label(row, text=f"[{k}]", bg=MID, fg=CYAN,
                     font=("Courier", 9, "bold"), width=8, anchor="w").pack(side=tk.LEFT)
            tk.Label(row, text=v, bg=MID, fg=GRAY,
                     font=("Courier", 9)).pack(side=tk.LEFT)

        # ── Status + buttons ──
        self.status_lbl = tk.Label(self.root, text="● STOPPED",
                                    bg=DARK, fg=RED,
                                    font=("Courier", 13, "bold"))
        self.status_lbl.pack(pady=(10, 6))

        btn_row = tk.Frame(self.root, bg=DARK)
        btn_row.pack()
        tk.Button(btn_row, text="  STOP  ", command=self._stop,
                  bg=RED, fg="#ffffff", font=("Courier", 11, "bold"),
                  relief=tk.FLAT, padx=10, pady=6).pack(side=tk.LEFT, padx=8)
        tk.Button(btn_row, text="  QUIT  ", command=self._quit,
                  bg=ACCENT, fg="#ffffff", font=("Courier", 10),
                  relief=tk.FLAT, padx=10, pady=6).pack(side=tk.LEFT, padx=8)

    # ── Internals ─────────────────────────────────────────────────────────────

    def _on_slider(self):
        self.node.vx    = self.vx_var.get()
        self.node.vy    = self.vy_var.get()
        self.node.omega = self.omega_var.get()
        self._refresh_labels()
        self._refresh_diagram()
        self._refresh_status()

    def _refresh_labels(self):
        self._vx_lbl.config(   text=f"{self.node.vx:+.2f} m/s")
        self._vy_lbl.config(   text=f"{self.node.vy:+.2f} m/s")
        self._omega_lbl.config(text=f"{self.node.omega:+.2f} r/s")

    def _refresh_diagram(self):
        wheels = swerve_kinematics(self.node.vx, self.node.vy, self.node.omega)
        self.diagram.update_wheels(wheels)

    def _refresh_status(self):
        vx, vy, om = self.node.vx, self.node.vy, self.node.omega
        if abs(vx) < 0.02 and abs(vy) < 0.02 and abs(om) < 0.02:
            self.status_lbl.config(text="● STOPPED", fg=RED)
            return
        parts = []
        if vx >  0.02: parts.append("FWD")
        if vx < -0.02: parts.append("REV")
        if vy >  0.02: parts.append("STRAFE←")
        if vy < -0.02: parts.append("STRAFE→")
        if om >  0.02: parts.append("↺ CCW")
        if om < -0.02: parts.append("↻ CW")
        self.status_lbl.config(text="● " + "  ".join(parts), fg=GREEN)

    # ── Keyboard smooth driving ────────────────────────────────────────────────

    _KEY_MAP = {
        'w': ('vx',     0.1),
        's': ('vx',    -0.1),
        'a': ('vy',     0.1),
        'd': ('vy',    -0.1),
        'q': ('omega',  0.15),
        'e': ('omega', -0.15),
    }

    def _key_press(self, event):
        key = event.keysym.lower()
        if key == 'space':
            self._stop(); return
        self._held.add(key)

    def _key_release(self, event):
        self._held.discard(event.keysym.lower())

    def _key_timer(self):
        """Poll held keys 20 Hz and nudge sliders."""
        for key, (axis, delta) in self._KEY_MAP.items():
            if key in self._held:
                if axis == 'vx':
                    v = max(-MAX_SPEED, min(MAX_SPEED, self.vx_var.get() + delta))
                    self.vx_var.set(v)
                elif axis == 'vy':
                    v = max(-MAX_SPEED, min(MAX_SPEED, self.vy_var.get() + delta))
                    self.vy_var.set(v)
                elif axis == 'omega':
                    v = max(-MAX_OMEGA, min(MAX_OMEGA, self.omega_var.get() + delta))
                    self.omega_var.set(v)
                self._on_slider()
        self.root.after(50, self._key_timer)   # 20 Hz

    # ── Controls ──────────────────────────────────────────────────────────────

    def _stop(self):
        self.vx_var.set(0.0)
        self.vy_var.set(0.0)
        self.omega_var.set(0.0)
        self._on_slider()

    def _quit(self):
        self._stop()
        self.root.quit()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# ─── Main ─────────────────────────────────────────────────────────────────────

def _spin_ros(node, executor):
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)
    except Exception as e:
        print(f"ROS error: {e}")


def main():
    rclpy.init()
    node = SwerveController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    threading.Thread(target=_spin_ros, args=(node, executor), daemon=True).start()

    print("Swerve Controller started")
    print("  W/S  : forward / back")
    print("  A/D  : strafe left / right")
    print("  Q/E  : rotate CCW / CW")
    print("  Space: stop")

    gui = SwerveGUI(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")


if __name__ == '__main__':
    main()