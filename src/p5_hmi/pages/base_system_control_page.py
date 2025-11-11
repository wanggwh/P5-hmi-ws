from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.slider import MDSlider
from kivy.clock import Clock
import math

class NonInteractiveSlider(MDSlider):
    """A slider that cannot be interacted with but shows values"""
    def on_touch_down(self, touch):
        return False  # Don't handle touch events
    
    def on_touch_move(self, touch):
        return False  # Don't handle touch events
        
    def on_touch_up(self, touch):
        return False  # Don't handle touch events

class BaseSystemControlPage(MDFloatLayout):
    robot_name = None  # Set in subclass

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.app = None
        self.update_event = None
        self.repeat_event = None
        self.repeat_joint_index = None
        self.repeat_direction = None
        self.repeat_interval = 0.3  # Start with 300ms interval
        self.repeat_count = 0

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

    def increment_joint(self, joint_index):
        """Increment joint angle by 1 degree"""
        slider_name = f'joint{joint_index+1}_slider'
        if hasattr(self.ids, slider_name):
            slider = getattr(self.ids, slider_name)
            new_value = min(slider.value + 1, slider.max)
            slider.value = new_value
            print(f"Incremented Joint {joint_index+1} to {new_value}°")

    def decrement_joint(self, joint_index):
        """Decrement joint angle by 1 degree"""
        slider_name = f'joint{joint_index+1}_slider'
        if hasattr(self.ids, slider_name):
            slider = getattr(self.ids, slider_name)
            new_value = max(slider.value - 1, slider.min)
            slider.value = new_value
            print(f"Decremented Joint {joint_index+1} to {new_value}°")

    def start_increment(self, joint_index):
        """Start incrementing joint angle with acceleration"""
        self.repeat_joint_index = joint_index
        self.repeat_direction = 1
        self.repeat_interval = 0.3
        self.repeat_count = 0
        # Immediate first increment
        self.increment_joint(joint_index)
        # Start repeat timer
        self.repeat_event = Clock.schedule_interval(self._repeat_action, self.repeat_interval)

    def start_decrement(self, joint_index):
        """Start decrementing joint angle with acceleration"""
        self.repeat_joint_index = joint_index
        self.repeat_direction = -1
        self.repeat_interval = 0.3
        self.repeat_count = 0
        # Immediate first decrement
        self.decrement_joint(joint_index)
        # Start repeat timer
        self.repeat_event = Clock.schedule_interval(self._repeat_action, self.repeat_interval)

    def stop_increment_decrement(self):
        """Stop the repeat action"""
        if self.repeat_event:
            self.repeat_event.cancel()
            self.repeat_event = None
        self.repeat_joint_index = None
        self.repeat_direction = None
        self.repeat_count = 0

    def _repeat_action(self, dt):
        """Internal method to handle repeated increment/decrement with acceleration"""
        if self.repeat_joint_index is None or self.repeat_direction is None:
            return False
        
        # Perform the action
        if self.repeat_direction == 1:
            self.increment_joint(self.repeat_joint_index)
        else:
            self.decrement_joint(self.repeat_joint_index)
        
        self.repeat_count += 1
        
        # Accelerate after some repetitions
        if self.repeat_count > 5 and self.repeat_interval > 0.05:
            # Cancel current event and start a faster one
            if self.repeat_event:
                self.repeat_event.cancel()
            self.repeat_interval = max(0.05, self.repeat_interval * 0.8)  # Speed up by 20%
            self.repeat_event = Clock.schedule_interval(self._repeat_action, self.repeat_interval)
            return False  # Stop this event, new one will continue
        
        return True  # Continue this event

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
            self.app.hmi_node.send_move_to_pre_def_pose_request(self.robot_name, "HOME")
            print(f"Sent HOME configuration request for {self.robot_name}")

    def move_to_home_specific_robot(self):
        print(f"Moving {self.robot_name} to its specific HOME position")
        # Default: just call move_to_home
        self.move_to_home()


class AliceSystemControlPage(BaseSystemControlPage):
    robot_name = "alice"
    def move_to_home_specific_robot(self):
        print("Moving ALICE to her very own HOME position!")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request("ALICE", "HOME_ALICE")
            print("Sent HOME_ALICE configuration request for ALICE")

class BobSystemControlPage(BaseSystemControlPage):
    robot_name = "bob"
    move_to_pose = "HOME"
    def move_to_home_specific_robot(self):
        print("Moving BOB to his very own HOME position!")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_move_to_pre_def_pose_request(self.robot_name, self.move_to_pose)
            print("Sent HOME_BOB configuration request for BOB")