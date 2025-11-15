from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.slider import MDSlider
from kivy.clock import Clock
import math


class AliceConfigurationSetup(MDFloatLayout):
    robot_name = "alice"
    robot_name_caps = "Alice"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.app = None


    def move_to_home_specific_robot(self):
        print("Moving ALICE to his very own HOME position!")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("alice", "ALICE_HOME")
            print("Sent HOME_BOB configuration request for BOB")

    def move_to_upright(self):
        print("Moving ALICE to UPRIGHT position")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("alice", "UPRIGHT")
            print("Sent UPRIGHT configuration request for ALICE")