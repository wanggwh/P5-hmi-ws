from kivymd.uix.floatlayout import MDFloatLayout
import math


class BobSystemControlPage(MDFloatLayout):
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
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_robot_configuration("HOME")
            print("Sent HOME configuration request")
        
    
    def move_to_home_bob(self):
        print("Moving bob to HOME position")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_robot_configuration("HOME_BOB")
            print("Sent HOME_BOB configuration request")

    

