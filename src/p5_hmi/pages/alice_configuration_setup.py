import json
import os
import time
from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.slider import MDSlider
from kivy.clock import Clock
from kivy.properties import ListProperty, StringProperty
from kivymd.uix.button import MDRaisedButton
from kivymd.app import MDApp
from kivy.animation import Animation

class AliceConfigurationSetup(MDFloatLayout):
    robot_name = "alice"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.app = MDApp.get_running_app()
        
        # Initialiser app storage for konfigurationer hvis det ikke findes
        if not hasattr(self.app, 'alice_saved_configurations'):
            self.app.alice_saved_configurations = []
            
        #Path til JSON fil med pre-defined poses
        self.poses_json_path = os.path.expanduser("~/Documents/P5-hmi-ws/config/pre_config_poses.json")

        # Load predefined poses fra JSON og opret knapper
        Clock.schedule_once(self.load_predefined_poses, 0.1)
                

        # Path to predefined poses JSON file
        self.app.hmi_node.receive_pose_configurations_data()  
        time.sleep(0.5)      
        self.pose_configuration_data = self.app.hmi_node.pass_pose_configuration_data()
        #print(self.pose_configuration_data) 
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
            
            # with open(self.poses_json_path, 'r') as f:
            #     all_poses = json.load(f)
            all_poses = json.loads(self.pose_configuration_data)
            print(all_poses)
            # Filter poses for ALICE robot (predefined = starts with ALICE_)
            alice_predefined_poses = {name: positions for name, positions in all_poses.items() 
                                   if name.startswith("ALICE_")}
            
            # Filter custom poses (ALICE poses without ALICE_ prefix)
            alice_custom_poses = {name: positions for name, positions in all_poses.items()
                               if not name.startswith("ALICE_") 
                               and not name.startswith("BOB_")
                               and name != "UPRIGHT"}
            
            print(f"Loaded {len(alice_predefined_poses)} predefined poses for ALICE")
            print(f"Loaded {len(alice_custom_poses)} custom configurations for ALICE")
            
            # Create buttons for predefined poses
            for pose_name in alice_predefined_poses.keys():
                self.create_predefined_pose_button(pose_name)
            
            # Store custom configuration names
            self.app.alice_saved_configurations = list(alice_custom_poses.keys())
            
            print("All predefined pose buttons created")
                
        except Exception as e:
            print(f"Error loading predefined poses: {e}")
            import traceback
            traceback.print_exc()

    def create_predefined_pose_button(self, pose_name):
        """Create button for a predefined pose"""
        # Format button text (remove ALICE_ prefix for display)
        display_name = pose_name.replace("ALICE_", "").replace("_", " ").title()
        
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
            print(f"Added button: {display_name}")
        else:
            print("Warning: predefined_poses_container not found in KV file")

    def send_predefined_pose(self, pose_name):
        """Send request to move to predefined pose"""
        if self.app and hasattr(self.app, 'hmi_node'):
            json_data = self.make_json_data(pose_name)
            self.app.hmi_node.call_load_raw_JSON_request(json_data, wait_for_service=True)
            print(f"Sent {pose_name} configuration request for ALICE")

    def make_json_data(self, pose_name):
        json_data = {
                        "alice_move_to_pose": 
                        {
                            "description": "",
                            "date": "",
                            "author": "",
                            "threads": 
                            [
                                {
                                    "name": "alice_thread",
                                    "robot_name": "alice",
                                    "commands": 
                                    [
                                        {
                                            "command": "c_move",
                                            "args": 
                                            {
                                                "config_name": pose_name
                                            }
                                        }
                                    ]
                                }
                            ]
                        }
                    }
        return json.dumps(json_data)
    
    def send_custom_configuration(self, config_name):
        print(config_name)
        if self.app and hasattr(self.app, 'hmi_node'):
            json_data = self.make_json_data(config_name)
            print(json_data)
            self.app.hmi_node.call_load_raw_JSON_request(json_data, wait_for_service=True)
            print(f"Sent {config_name} configuration request for ALICE")

    def restore_saved_configurations(self, dt):
        """Gendan alle gemte konfigurationer når siden loades"""
        if hasattr(self.app, 'alice_saved_configurations'):
            for config_name in self.app.alice_saved_configurations:
                self.create_custom_config_button(config_name)

    def alice_update_joint_positions(self, joint_positions):
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
            
    def move_to_home(self):
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("alice", "ALICE_HOME")
            print("Sent HOME_ALICE configuration request for ALICE")

    def move_to_upright(self):
        print("Moving ALICE to UPRIGHT position")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("alice", "UPRIGHT")
            print("Sent UPRIGHT configuration request for ALICE")

    def save_configuration(self):
        config_name = self.ids.configuration_name_field.text.strip()
        
        if config_name and config_name not in self.app.alice_saved_configurations:
            # Gem konfigurationen i app storage
            self.app.alice_saved_configurations.append(config_name)
            print(f"Saved configuration: {config_name}")
            
            self.create_custom_config_button(config_name)
            
            # Clear tekstfeltet
            self.ids.configuration_name_field.text = ""
            self.app.hmi_node.save_pre_def_pose_request("alice", config_name)
        else:
            print("Please enter a valid, unique configuration name")
    
    def create_custom_config_button(self, config_name): 
        button = MDRaisedButton(
            text=config_name,
            size_hint=(None, None),
            size=("120dp", "50dp"),
            md_bg_color=self.app.theme_cls.primary_color if self.app else (0.2, 0.6, 1.0, 1),
            on_release=lambda x: self.send_custom_configuration(config_name)
        )
        
        # Tilføj knappen til custom configurations container
        self.ids.custom_config_container.add_widget(button)
    
    def restore_saved_configurations(self, dt):
        """Gendan alle gemte konfigurationer når siden loades"""
        if hasattr(self.app, 'alice_saved_configurations'):
            for config_name in self.app.alice_saved_configurations:
                self.create_custom_config_button(config_name)

    def alice_update_joint_positions(self, joint_positions):
        """Update joint position labels with current robot positions"""
        joint_label_ids = ['joint1_label', 'joint2_label', 'joint3_label', 
                        'joint4_label', 'joint5_label', 'joint6_label']
        
        for i, label_id in enumerate(joint_label_ids):
            if i < len(joint_positions) and hasattr(self.ids, label_id):
                value = joint_positions[i]
                if abs(value) < 0.05:  
                    value = 0.0
                self.ids[label_id].text = f"{value:.1f} deg"


            