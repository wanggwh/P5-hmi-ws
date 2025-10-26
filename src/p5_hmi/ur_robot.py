#!/usr/bin/env python3
import rtde_control
import rtde_receive
import time
import threading
import math
from typing import List, Optional, Dict

class UR5eRobot:
    def __init__(self, robot_ip: str = "192.168.1.2"):
        self.robot_ip = robot_ip
        self.rtde_c = None
        self.rtde_r = None
        self.connected = False
        
        # UR5e specific limits (degrees)
        self.joint_limits = [
            (-360, 360),  # Joint 1
            (-360, 360),  # Joint 2 
            (-360, 360),  # Joint 3
            (-360, 360),  # Joint 4
            (-360, 360),  # Joint 5
            (-360, 360)   # Joint 6
        ]
        
        # UR5e predefined positions (degrees)
        self.positions = {
            'home': [0, -90, 0, -90, 0, 0],
            'safe': [0, -90, -90, -90, 0, 0],
            'vertical': [0, -90, -90, -180, 0, 0],
            'pick_ready': [45, -60, -90, -120, 90, 0]
        }
        
        # UR5e safe speeds/accelerations
        self.safe_joint_speed = 0.5     # rad/s
        self.safe_joint_accel = 0.3     # rad/s²
        self.safe_tcp_speed = 0.1       # m/s
        self.safe_tcp_accel = 0.3       # m/s²
    
    def connect(self) -> bool:
        """Connect to UR5e robot"""
        try:
            print(f"Connecting to UR5e at {self.robot_ip}...")
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.connected = True
            
            # Verify robot model if possible
            robot_model = self.get_robot_model()
            print(f"Connected to robot model: {robot_model}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to UR5e: {e}")
            return False
    
    def get_robot_model(self) -> str:
        """Try to identify robot model"""
        try:
            # This is a simple check - UR5e typically has specific characteristics
            return "UR5e (assumed)"
        except:
            return "Unknown"
    
    def move_to_position(self, position_name: str):
        """Move to predefined UR5e position"""
        if position_name in self.positions and self.connected:
            joint_angles_deg = self.positions[position_name]
            joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
            
            print(f"Moving UR5e to {position_name} position")
            self.rtde_c.moveJ(joint_angles_rad, self.safe_joint_speed, self.safe_joint_accel)
    
    def check_joint_limits(self, joint_positions_deg: List[float]) -> bool:
        """Check if joint positions are within UR5e limits"""
        for i, (pos, (min_limit, max_limit)) in enumerate(zip(joint_positions_deg, self.joint_limits)):
            if pos < min_limit or pos > max_limit:
                print(f"Joint {i+1} position {pos}° exceeds limits ({min_limit}°, {max_limit}°)")
                return False
        return True
    
    def move_joints_safe(self, joint_positions_deg: List[float]):
        """Move joints with safety checks"""
        if not self.connected:
            print("Robot not connected!")
            return False
            
        if not self.check_joint_limits(joint_positions_deg):
            print("Joint positions exceed UR5e limits!")
            return False
        
        # Convert to radians
        joint_positions_rad = [math.radians(angle) for angle in joint_positions_deg]
        
        try:
            self.rtde_c.moveJ(joint_positions_rad, self.safe_joint_speed, self.safe_joint_accel)
            return True
        except Exception as e:
            print(f"Failed to move UR5e joints: {e}")
            return False
    
    def get_safety_status(self) -> Dict:
        """Get UR5e safety status"""
        if not self.connected:
            return {}
            
        try:
            return {
                'robot_mode': self.rtde_r.getRobotMode(),
                'safety_mode': self.rtde_r.getSafetyMode(),
                'is_program_running': self.rtde_r.isProgramRunning(),
                'is_protective_stopped': self.rtde_r.isProtectiveStopped(),
                'is_emergency_stopped': self.rtde_r.isEmergencyStopped()
            }
        except:
            return {}