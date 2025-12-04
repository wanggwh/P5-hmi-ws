import json
import os
from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.slider import MDSlider
from kivy.clock import Clock
from kivy.properties import ListProperty, StringProperty
from kivymd.uix.button import MDRaisedButton
from kivymd.app import MDApp
from kivy.animation import Animation

class BobConfigurationSetup(MDFloatLayout):
    robot_name = "bob"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.app = MDApp.get_running_app()
        
        # Initialiser app storage for konfigurationer hvis det ikke findes
        if not hasattr(self.app, 'bob_saved_configurations'):
            self.app.bob_saved_configurations = []

        # Path to predefined poses JSON file
        self.app.hmi_node.receive_pose_configurations_data()        

        # Gendan gemte konfigurationer når siden loades
        Clock.schedule_once(self.restore_saved_configurations, 0.2)

    def load_predefined_poses(self, dt):
        """Load predefined poses from JSON and create buttons"""
        try:
            if not os.path.exists(self.poses_json_path):
                print(f"Predefined poses file not found: {self.poses_json_path}")
                return
            
            # Vent til app.colors er klar
            if not hasattr(self.app, 'colors') or not self.app.colors:
                print("App colors not ready, retrying in 0.1s...")
                Clock.schedule_once(self.load_predefined_poses, 0.1)
                return
            
            with open(self.poses_json_path, 'r') as f:
                all_poses = json.load(f)
            
            # Filter poses for BOB robot (predefined = starts with BOB_)
            bob_predefined_poses = {name: positions for name, positions in all_poses.items() 
                                   if name.startswith("BOB_")}
            
            # Filter custom poses (BOB poses without BOB_ prefix)
            bob_custom_poses = {name: positions for name, positions in all_poses.items()
                               if not name.startswith("BOB_") 
                               and not name.startswith("ALICE_")
                               and name != "UPRIGHT"}
            
            print(f"Loaded {len(bob_predefined_poses)} predefined poses for BOB")
            print(f"Loaded {len(bob_custom_poses)} custom configurations for BOB")
            
            # Create buttons for predefined poses
            for pose_name in bob_predefined_poses.keys():
                self.create_predefined_pose_button(pose_name)
            
            # Store custom configuration names
            self.app.bob_saved_configurations = list(bob_custom_poses.keys())
            
            print("✅ All predefined pose buttons created")
                
        except Exception as e:
            print(f"Error loading predefined poses: {e}")
            import traceback
            traceback.print_exc()

    def create_predefined_pose_button(self, pose_name):
        """Create button for a predefined pose"""
        # Format button text (remove BOB_ prefix for display)
        display_name = pose_name.replace("BOB_", "").replace("_", " ").title()
        
        # Safely get colors with fallback values
        primary_color = (0.2, 0.6, 1.0, 1)
        text_color = (1, 1, 1, 1)
        
        if hasattr(self.app, 'colors') and self.app.colors:
            primary_color = self.app.colors.get('primary', primary_color)
            text_color = self.app.colors.get('text_light', text_color)
        
        button = MDRaisedButton(
            text=display_name,
            size_hint=(None, None),
            size=("140dp", "50dp"),
            md_bg_color=primary_color,
            text_color=text_color,
            on_release=lambda x, name=pose_name: self.send_predefined_pose(name)
        )
        
        # Tilføj til predefined poses container
        if hasattr(self.ids, 'predefined_poses_container'):
            self.ids.predefined_poses_container.add_widget(button)
            print(f"✅ Added button: {display_name}")
        else:
            print("Warning: predefined_poses_container not found in KV file")

    def send_predefined_pose(self, pose_name):
        """Send request to move to predefined pose"""
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("bob", pose_name)
            print(f"Sent {pose_name} configuration request for BOB")

    def save_configuration(self):
        """Save custom configuration to JSON file"""
        config_name = self.ids.configuration_name_field.text.strip()
        
        if not config_name:
            print("Please enter a configuration name")
            return
            
        if config_name in self.app.bob_saved_configurations:
            print("Configuration name already exists")
            return
        
        try:
            # Get current joint positions from HMI node
            if not (self.app and hasattr(self.app, 'hmi_node') 
                   and hasattr(self.app.hmi_node, 'bob_joint_positions')):
                print("❌ Current joint positions not available")
                return
            
            joint_positions = self.app.hmi_node.bob_joint_positions
            
            # Load existing JSON file
            with open(self.poses_json_path, 'r') as f:
                all_poses = json.load(f)
            
            # Add new custom configuration
            all_poses[config_name] = joint_positions
            
            # Save back to JSON file
            with open(self.poses_json_path, 'w') as f:
                json.dump(all_poses, f, indent=2)
            
            # Update app storage
            self.app.bob_saved_configurations.append(config_name)
            
            # Create button
            self.create_custom_config_button(config_name)
            
            # Clear input field
            self.ids.configuration_name_field.text = ""
            
            # Send ROS2 save request
            self.app.hmi_node.save_pre_def_pose_request("bob", config_name)
            
            print(f"✅ Saved configuration '{config_name}' to JSON: {joint_positions}")
            
        except Exception as e:
            print(f"❌ Error saving configuration: {e}")
            import traceback
            traceback.print_exc()
    
    def create_custom_config_button(self, config_name):
        # Safely get colors with fallback
        accent_color = (1.0, 0.5, 0.3, 1)
        text_color = (1, 1, 1, 1)
        
        if hasattr(self.app, 'colors') and self.app.colors:
            accent_color = self.app.colors.get('accent_coral', accent_color)
            text_color = self.app.colors.get('text_light', text_color)
        
        button = MDRaisedButton(
            text=config_name,
            size_hint=(None, None),
            size=("120dp", "50dp"),
            md_bg_color=accent_color,
            text_color=text_color,
            on_release=lambda x: self.send_custom_configuration(config_name)
        )
        
        # Tilføj knappen til custom configurations container
        if hasattr(self.ids, 'custom_config_container'):
            self.ids.custom_config_container.add_widget(button)
        else:
            print("Warning: custom_config_container not found in KV file")
    
    def send_custom_configuration(self, config_name):
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("bob", str(config_name))
            print(f"Sent {config_name} configuration request for BOB")

    def restore_saved_configurations(self, dt):
        """Gendan alle gemte konfigurationer når siden loades"""
        if hasattr(self.app, 'bob_saved_configurations'):
            for config_name in self.app.bob_saved_configurations:
                self.create_custom_config_button(config_name)

    def bob_update_joint_positions(self, joint_positions):
        """Update joint position labels with current robot positions"""
        joint_label_ids = ['joint1_label', 'joint2_label', 'joint3_label', 
                        'joint4_label', 'joint5_label', 'joint6_label']
        
        for i, label_id in enumerate(joint_label_ids):
            if i < len(joint_positions) and hasattr(self.ids, label_id):
                value = joint_positions[i]
                if abs(value) < 0.05:  
                    value = 0.0
                self.ids[label_id].text = f"{value:.1f} deg"


    def get_pre_def_poses(self):
        if self.app and hasattr(self.app, 'hmi_node'):
            pre_def_poses = self.app.hmi_node.receive_pose_configurations_data()
            print("OKAYYY")
            print(pre_def_poses)