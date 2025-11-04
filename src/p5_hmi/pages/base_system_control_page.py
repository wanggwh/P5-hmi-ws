from kivymd.uix.floatlayout import MDFloatLayout
import math

class BaseSystemControlPage(MDFloatLayout):
    robot_name = None  # Set in subclass

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.app = None
        self.update_event = None

    def on_parent(self, widget, parent):
        if parent:
            from kivymd.app import MDApp
            self.app = MDApp.get_running_app()

    def on_joint_change(self, joint_index, value_degrees):
        value_int = int(value_degrees)
        print(f"Joint {joint_index+1} changed to {value_int}°")
        if hasattr(self.ids, f'joint{joint_index+1}_value'):
            getattr(self.ids, f'joint{joint_index+1}_value').text = f"{value_int}°"
        self.publish_joint_angles()

    def publish_joint_angles(self):
        if not self.app or not hasattr(self.app, 'hmi_node'):
            return
        try:
            joint_positions = []
            for i in range(1, 7):
                slider_name = f'joint{i}_slider'
                if hasattr(self.ids, slider_name):
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
        print(f"Moving {self.robot_name} to HOME position")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_robot_configuration(self.robot_name, "HOME")
            print(f"Sent HOME configuration request for {self.robot_name}")

    def move_to_home_specific_robot(self):
        print(f"Moving {self.robot_name} to its specific HOME position")
        # Default: just call move_to_home
        self.move_to_home()


class AliceSystemControlPage(BaseSystemControlPage):
    robot_name = "ALICE"
    def move_to_home_specific_robot(self):
        print("Moving ALICE to her very own HOME position!")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_robot_configuration("ALICE", "HOME_ALICE")
            print("Sent HOME_ALICE configuration request for ALICE")

class BobSystemControlPage(BaseSystemControlPage):
    robot_name = "BOB"
    def move_to_home_specific_robot(self):
        print("Moving BOB to his very own HOME position!")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_robot_configuration("BOB", "HOME_BOB")
            print("Sent HOME_BOB configuration request for BOB")