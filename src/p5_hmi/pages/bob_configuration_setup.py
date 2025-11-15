from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.slider import MDSlider
from kivy.clock import Clock
from kivy.properties import ListProperty, StringProperty
from kivymd.uix.button import MDRaisedButton


class BobConfigurationSetup(MDFloatLayout):
    saved_configurations = ListProperty([])
    robot_name = "bob"
    robot_name_caps = "BOB"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.app = None

    def move_to_home(self):
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("BOB", "BOB_HOME")
            print("Sent HOME_BOB configuration request for BOB")

    def move_to_upright(self):
        print("Moving BOB to UPRIGHT position")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("BOB", "UPRIGHT")
            print("Sent UPRIGHT configuration request for BOB")

    def save_configuration(self):
        config_name = self.ids.configuration_name_field.text.strip()
        
        if config_name and config_name not in self.saved_configurations:
            # Gem konfigurationen
            self.saved_configurations.append(config_name)
            print(f"Saved configuration: {config_name}")
            
            self.create_custom_config_button(config_name)
            
            # Clear tekstfeltet
            self.ids.configuration_name_field.text = ""
        else:
            print("Please enter a valid, unique configuration name")
    
    def create_custom_config_button(self, config_name):
        # Find custom configurations container (du skal give den et ID i KV)
        # Opret ny knap
        button = MDRaisedButton(
            text=config_name,
            size_hint=(None, None),
            size=("120dp", "50dp"),
            md_bg_color=self.app.colors['primary'] if self.app else (0.2, 0.6, 1.0, 1),
            on_release=lambda x: self.load_custom_configuration(config_name)
        )
        
        # Tilføj knappen til custom configurations container
        self.ids.custom_config_container.add_widget(button)
    
    def load_custom_configuration(self, config_name):
        print(f"Loading configuration: {config_name}")
        # Implementer load logik her