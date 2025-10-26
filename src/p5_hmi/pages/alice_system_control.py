from kivymd.uix.floatlayout import MDFloatLayout
from kivy.clock import Clock
import math


class AliceSystemControlPage(MDFloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.app = None
        self.update_event = None

    def on_parent(self, widget, parent):
        if parent:
            from kivymd.app import MDApp
            self.app = MDApp.get_running_app()
    
    def on_joint_change(self, joint_index, value_degrees):
        """Handle joint slider changes"""
        value_int = int(value_degrees)
        print(f"Joint {joint_index+1} changed to {value_int}°")
        
        # Update value label
        if hasattr(self.ids, f'joint{joint_index+1}_value'):
            getattr(self.ids, f'joint{joint_index+1}_value').text = f"{value_int}°"
            
        # Publish joint angles via HMI node
        self.publish_joint_angles()

    def publish_joint_angles(self):
        """Publish current joint angles to ROS2 via HMI node"""
        if not self.app or not hasattr(self.app, 'hmi_node'):
            return
            
        try:
            joint_positions = []
            
            for i in range(1, 7):
                slider_name = f'joint{i}_slider'
                if hasattr(self.ids, slider_name):
                    # Convert degrees to radians
                    degrees = getattr(self.ids, slider_name).value
                    radians = math.radians(degrees)
                    joint_positions.append(radians)
                else:
                    joint_positions.append(0.0)
            
            self.app.hmi_node.publish_joint_states(joint_positions)
            print(f"Published joint angles: {[math.degrees(p) for p in joint_positions]}")
            
        except Exception as e:
            print(f"Failed to publish joint angles: {e}")
    
    def move_to_home(self):
        """Move robot to home position"""
        print("Moving UR5e to HOME position")
        home_positions = [0, -90, 0, -90, 0, 0]
        slider_names = ['joint1_slider', 'joint2_slider', 'joint3_slider', 
                       'joint4_slider', 'joint5_slider', 'joint6_slider']
        value_names = ['joint1_value', 'joint2_value', 'joint3_value',
                      'joint4_value', 'joint5_value', 'joint6_value']
        
        for i, (pos, slider_name, value_name) in enumerate(zip(home_positions, slider_names, value_names)):
            if hasattr(self.ids, slider_name):
                getattr(self.ids, slider_name).value = pos
            if hasattr(self.ids, value_name):
                getattr(self.ids, value_name).text = f"{pos}°"
                
        # Publish home position
        self.publish_joint_angles()
        
        # Publish "HOME" command on /robot_commands topic via HMINode
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.publish_robot_command("HOME")
            print("Published 'HOME' on /robot_commands")
    
    def move_to_home_alice(self):
        """Move Alice robot to home position"""
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.publish_robot_command("HOME_ALICE")
            print("Published 'HOME_ALICE' on /robot_commands")

