#!/usr/bin/env python3
"""
SAUVC Simulation Dashboard & Interactive Control Panel
Provides Start, Stop (Pause), Reset, and Manual / Autonomous control buttons
along with real-time status and live camera view integration.
"""

import sys
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

class SauvcControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("SAUVC Mission Control Panel")
        self.geometry("460x480")
        self.resizable(False, False)
        self.configure(bg="#121820")

        # ROS 2 Node
        rclpy.init(args=None)
        self.node = Node("sauvc_sim_control_panel")
        self.start_pub = self.node.create_publisher(Bool, "/sauvc/sim/start", 10)
        self.stop_pub = self.node.create_publisher(Bool, "/sauvc/sim/stop", 10)
        self.reset_pub = self.node.create_publisher(Bool, "/sauvc/sim/reset", 10)
        self.mode_pub = self.node.create_publisher(String, "/sauvc/control_mode", 10)
        self.cmd_pub = self.node.create_publisher(Twist, "/sauvc/cmd_vel", 10)

        self.node.create_subscription(String, "/sauvc/mission_status", self.status_cb, 10)

        self.build_ui()
        self.poll_ros()

    def build_ui(self):
        header = tk.Label(
            self, text="SAUVC SIMULATION DASHBOARD",
            font=("Helvetica", 14, "bold"), fg="#00e5ff", bg="#121820", pady=10
        )
        header.pack()

        # Status Display Frame
        status_frame = tk.LabelFrame(
            self, text=" Mission Telemetry ", fg="#a0aec0", bg="#1a202c",
            font=("Helvetica", 9, "bold"), padx=10, pady=8
        )
        status_frame.pack(fill="x", padx=16, pady=5)

        self.status_var = tk.StringVar(value="Waiting for mission status...")
        self.status_lbl = tk.Label(
            status_frame, textvariable=self.status_var, wraplength=400,
            justify="left", fg="#68d391", bg="#1a202c", font=("Helvetica", 10)
        )
        self.status_lbl.pack(fill="x")

        # Control Buttons Frame
        btn_frame = tk.LabelFrame(
            self, text=" Simulation Master Controls ", fg="#a0aec0", bg="#1a202c",
            font=("Helvetica", 9, "bold"), padx=10, pady=10
        )
        btn_frame.pack(fill="x", padx=16, pady=10)

        # Start Button
        self.btn_start = tk.Button(
            btn_frame, text="▶  START / RESUME", bg="#276749", fg="white",
            activebackground="#2f855a", activeforeground="white",
            font=("Helvetica", 11, "bold"), height=2, command=self.on_start
        )
        self.btn_start.pack(fill="x", pady=4)

        # Stop Button
        self.btn_stop = tk.Button(
            btn_frame, text="⏸  STOP / PAUSE", bg="#9b2c2c", fg="white",
            activebackground="#c53030", activeforeground="white",
            font=("Helvetica", 11, "bold"), height=2, command=self.on_stop
        )
        self.btn_stop.pack(fill="x", pady=4)

        # Reset Button
        self.btn_reset = tk.Button(
            btn_frame, text="🔄  RESET SIMULATION", bg="#2b6cb0", fg="white",
            activebackground="#3182ce", activeforeground="white",
            font=("Helvetica", 11, "bold"), height=2, command=self.on_reset
        )
        self.btn_reset.pack(fill="x", pady=4)

        # Teleoperation & Mode Frame
        tele_frame = tk.LabelFrame(
            self, text=" Manual Steering Controls ", fg="#a0aec0", bg="#1a202c",
            font=("Helvetica", 9, "bold"), padx=10, pady=8
        )
        tele_frame.pack(fill="x", padx=16, pady=5)

        grid_box = tk.Frame(tele_frame, bg="#1a202c")
        grid_box.pack()

        tk.Button(grid_box, text="Forward (W)", bg="#4a5568", fg="white", width=12, command=lambda: self.send_twist(0.3, 0, 0, 0)).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(grid_box, text="Turn Left (A)", bg="#4a5568", fg="white", width=12, command=lambda: self.send_twist(0.15, 0, 0, 0.4)).grid(row=1, column=0, padx=2, pady=2)
        tk.Button(grid_box, text="Halt (S)", bg="#742a2a", fg="white", width=12, command=lambda: self.send_twist(0, 0, 0, 0)).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(grid_box, text="Turn Right (D)", bg="#4a5568", fg="white", width=12, command=lambda: self.send_twist(0.15, 0, 0, -0.4)).grid(row=1, column=2, padx=2, pady=2)
        tk.Button(grid_box, text="Submerge (Z)", bg="#2d3748", fg="white", width=12, command=lambda: self.send_twist(0, 0, -0.2, 0)).grid(row=2, column=0, padx=2, pady=2)
        tk.Button(grid_box, text="Surface (X)", bg="#2d3748", fg="white", width=12, command=lambda: self.send_twist(0, 0, 0.2, 0)).grid(row=2, column=2, padx=2, pady=2)

    def on_start(self):
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)

    def on_stop(self):
        msg = Bool()
        msg.data = True
        self.stop_pub.publish(msg)

    def on_reset(self):
        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)

    def send_twist(self, vx, vy, vz, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.linear.z = float(vz)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def status_cb(self, msg):
        self.status_var.set(msg.data)

    def poll_ros(self):
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.01)
                self.after(20, self.poll_ros)
        except Exception:
            self.on_closing()

    def on_closing(self):
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        self.destroy()

if __name__ == "__main__":
    import signal
    app = SauvcControlPanel()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)

    def sig_handler(sig, frame):
        app.on_closing()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        app.mainloop()
    except (KeyboardInterrupt, SystemExit):
        app.on_closing()

