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
        self.app = MDApp.get_running_app() #Reference til hovedappen
        
        # Initialiser app storage for konfigurationer hvis det ikke findes
        if not hasattr(self.app, 'bob_saved_configurations'):
            self.app.bob_saved_configurations = []
        
        # Gendan gemte konfigurationer når siden loades
        Clock.schedule_once(self.restore_saved_configurations, 0.1)

    def move_to_home(self):
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("bob", "BOB_HOME")
            print("Sent HOME_BOB configuration request for BOB")

    def move_to_upright(self):
        print("Moving BOB to UPRIGHT position")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("bob", "UPRIGHT")
            print("Sent UPRIGHT configuration request for BOB")

    def save_configuration(self):
        config_name = self.ids.configuration_name_field.text.strip()
        
        if config_name and config_name not in self.app.bob_saved_configurations:
            # Gem konfigurationen i app storage
            self.app.bob_saved_configurations.append(config_name)
            print(f"Saved configuration: {config_name}")
            
            self.create_custom_config_button(config_name)
            
            # Clear tekstfeltet
            self.ids.configuration_name_field.text = ""
            self.app.hmi_node.save_pre_def_pose_request("bob", config_name)
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
                # Format til 2 decimaler
                self.ids[label_id].text = f"{joint_positions[i]:.2f}"

