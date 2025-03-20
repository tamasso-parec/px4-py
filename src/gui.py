import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, BatteryStatus, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, FailsafeFlags, VehicleStatus, VehicleAttitudeSetpoint, SensorGps
from commander_msg.msg import CommanderAll, CommanderPathPoint, CommanderArm, CommanderMode, CommanderAction
import numpy as np
import json
import math

class GUI(Node):
    def __init__(self, root):
        super().__init__('commander_gui')
        self.root = root
        self.root.title("Commander GUI")
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.commander_arm_pub = self.create_publisher(
            CommanderArm, '/com/in/arm', qos_profile)
        self.commander_mode_pub = self.create_publisher(
            CommanderMode, '/com/in/mode', qos_profile)
        
        # Speed Sliders
        self.speed_var_1 = tk.DoubleVar(value=0.2)
        self.speed_var_2 = tk.DoubleVar(value=0.2)
        ttk.Label(root, text="Speed 1:").grid(row=0, column=0)
        ttk.Scale(root, from_=0, to=3, orient="horizontal", variable=self.speed_var_1).grid(row=0, column=1)
        
        ttk.Label(root, text="Speed 2:").grid(row=1, column=0)
        ttk.Scale(root, from_=0, to=3, orient="horizontal", variable=self.speed_var_2).grid(row=1, column=1)
        
        # Movement Buttons
        self.btn_forward = ttk.Button(root, text="↑", command=lambda: self.manual_control("pad_forward"))
        self.btn_left = ttk.Button(root, text="←", command=lambda: self.manual_control("pad_left"))
        self.btn_right = ttk.Button(root, text="→", command=lambda: self.manual_control("pad_right"))
        self.btn_back = ttk.Button(root, text="↓", command=lambda: self.manual_control("pad_back"))
        
        self.btn_forward.grid(row=2, column=1)
        self.btn_left.grid(row=3, column=0)
        self.btn_right.grid(row=3, column=2)
        self.btn_back.grid(row=4, column=1)
        
        # Status Labels
        self.status_label = ttk.Label(root, text="Drone: UNKNOWN")
        self.status_label.grid(row=5, column=0, columnspan=3)
        
        # Control Buttons
        self.arm_button = ttk.Button(root, text="ARM", command=self.arm)
        self.disarm_button = ttk.Button(root, text="DISARM", command=self.disarm)
        self.arm_button.grid(row=6, column=0)
        self.disarm_button.grid(row=6, column=1)
        
        self.takeoff_button = ttk.Button(root, text="TAKEOFF", command=self.takeoff)
        self.land_button = ttk.Button(root, text="LAND", command=self.land)
        self.takeoff_button.grid(row=7, column=0)
        self.land_button.grid(row=7, column=1)
        
        # Mode Selection
        self.mode_var = tk.StringVar(value="None")
        ttk.Label(root, text="Mode:").grid(row=9, column=0)
        self.mode_menu = ttk.Combobox(root, textvariable=self.mode_var, values=["spin", "routine", "goto", "path", "updown", "None"])
        self.mode_menu.grid(row=9, column=1)
        
        self.confirm_button = ttk.Button(root, text="Confirm Mode", command=self.confirm_mode)
        self.confirm_button.grid(row=10, column=0, columnspan=2)
        
    def arm(self):
        msg = CommanderArm()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.arm = True
        self.commander_arm_pub.publish(msg)
        self.status_label.config(text="Drone: ARMED")
    
    def disarm(self):
        msg = CommanderArm()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.arm = False
        self.commander_arm_pub.publish(msg)
        self.status_label.config(text="Drone: DISARMED")
    
    def takeoff(self):
        print("TAKEOFF command sent")
    
    def land(self):
        print("LAND command sent")
    
    def confirm_mode(self):
        msg = CommanderMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.mode = self.mode_var.get()
        self.commander_mode_pub.publish(msg)
        print(f"Mode set to {self.mode_var.get()}")
    
    def manual_control(self, action):
        print(f"Manual control: {action}")
    
    def vehicle_status_callback(self, msg):
        arm_status = "ARMED" if msg.arming_state == 2 else "DISARMED"
        self.status_label.config(text=f"Drone: {arm_status}")
    
if __name__ == "__main__":
    rclpy.init()
    root = tk.Tk()
    gui = GUI(root)
    root.mainloop()
    rclpy.shutdown()
