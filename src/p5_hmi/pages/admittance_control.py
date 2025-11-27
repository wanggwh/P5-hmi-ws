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

class AdmittanceControl(MDFloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.app = None
        self.bob_admittance_enabled = False  # Default state for BOB - DISABLED
        self.alice_admittance_enabled = False  # Default state for ALICE - DISABLED
        
        # Slider control variables
        self.repeat_event = None
        self.repeat_slider_index = None
        self.repeat_direction = None
        self.repeat_interval = 0.3
        self.repeat_count = 0
        self.publish_debounce_event = None  # For debouncing parameter publishing

    def on_parent(self, widget, parent):
        """Called when the widget is added to a parent"""
        if parent:
            from kivymd.app import MDApp
            self.app = MDApp.get_running_app()

    def on_admittance_bob_toggle(self, button, active):
        """Called when BOB segmented button state changes"""
        if active:
            if button.text == "Enable":
                self.enable_bob_admittance_control()
            elif button.text == "Disable":
                self.disable_bob_admittance_control()

    def on_admittance_alice_toggle(self, button, active):
        """Called when ALICE segmented button state changes"""
        if active:
            if button.text == "Enable":
                self.enable_alice_admittance_control()
            elif button.text == "Disable":
                self.disable_alice_admittance_control()

    def enable_bob_admittance_control(self):
        """Enable admittance control for BOB"""
        print("BOB Admittance Control ENABLE requested")
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_set_admittance_status_request("bob", True, 250)
            print("Sent enable request for BOB admittance control")

    def disable_bob_admittance_control(self):
        """Disable admittance control for BOB"""
        print("BOB Admittance Control DISABLE requested")
        if self.app and hasattr(self.app, 'hmi_node'):
            # Send ROS2 service request - fjern 'self' som første parameter
            self.app.hmi_node.send_set_admittance_status_request("bob", False, 250)
            print("Sent DISABLE request for BOB admittance control")

    def enable_alice_admittance_control(self):
        """Enable admittance control for ALICE"""
        print("ALICE Admittance Control ENABLE requested")
        if self.app and hasattr(self.app, 'hmi_node'):
            # Send ROS2 service request - fjern 'self' som første parameter  
            self.app.hmi_node.send_set_admittance_status_request("alice", True, 250)
            print("Sent enable request for ALICE admittance control")

    def disable_alice_admittance_control(self):
        """Disable admittance control for ALICE"""
        print("ALICE Admittance Control DISABLE requested")
        if self.app and hasattr(self.app, 'hmi_node'):
            # Send ROS2 service request - fjern 'self' som første parameter
            self.app.hmi_node.send_set_admittance_status_request("alice", False, 250)
            print("Sent DISABLE request for ALICE admittance control")

    def get_bob_status(self):
        """Get current BOB admittance control status"""
        return self.bob_admittance_enabled

    def get_alice_status(self):
        """Get current ALICE admittance control status"""
        return self.alice_admittance_enabled

    # Slider control functions
    def on_joint_change(self, slider_index, value):
        """Called when any slider value changes"""
        value_float = round(float(value), 1)
        slider_names = ["M", "D", "k"]
        slider_name = slider_names[slider_index]
        
        # Update the corresponding label immediately for instant feedback
        if hasattr(self.ids, f'{slider_name}_value'):
            getattr(self.ids, f'{slider_name}_value').text = f"{value_float}"
        
        # Debounce publishing - only publish after 0.3s of no changes
        if self.publish_debounce_event:
            self.publish_debounce_event.cancel()
        
        self.publish_debounce_event = Clock.schedule_once(
            lambda dt: self.publish_admittance_parameters(), 0.3)

    def publish_admittance_parameters(self):
        """Publish admittance control parameters to ROS2"""
        if not self.app or not hasattr(self.app, 'hmi_node'):
            return
        try:
            # Get current slider values - direct access is faster
            M_value = self.ids.M_slider.value
            D_value = self.ids.D_slider.value
            k_value = self.ids.k_slider.value
            
            # Send to ROS2
            # self.app.hmi_node.publish_admittance_parameters(M_value, D_value, k_value)
        except Exception as e:
            print(f"Failed to publish admittance parameters: {e}")

    def increment_parameter(self, slider_index):
        """Increment parameter by step size"""
        slider_names = ["M_slider", "D_slider", "k_slider"]
        slider_name = slider_names[slider_index]
        if hasattr(self.ids, slider_name):
            slider = getattr(self.ids, slider_name)
            new_value = min(slider.value + slider.step, slider.max)
            slider.value = new_value

    def decrement_parameter(self, slider_index):
        """Decrement parameter by step size"""
        slider_names = ["M_slider", "D_slider", "k_slider"]
        slider_name = slider_names[slider_index]
        if hasattr(self.ids, slider_name):
            slider = getattr(self.ids, slider_name)
            new_value = max(slider.value - slider.step, slider.min)
            slider.value = new_value

    def start_increment(self, slider_index):
        """Start incrementing parameter with acceleration"""
        self.repeat_slider_index = slider_index
        self.repeat_direction = 1
        self.repeat_interval = 0.3
        self.repeat_count = 0
        # Immediate first increment
        self.increment_parameter(slider_index)
        # Start repeat timer
        self.repeat_event = Clock.schedule_interval(self._repeat_action, self.repeat_interval)

    def start_decrement(self, slider_index):
        """Start decrementing parameter with acceleration"""
        self.repeat_slider_index = slider_index
        self.repeat_direction = -1
        self.repeat_interval = 0.3
        self.repeat_count = 0
        # Immediate first decrement
        self.decrement_parameter(slider_index)
        # Start repeat timer
        self.repeat_event = Clock.schedule_interval(self._repeat_action, self.repeat_interval)

    def stop_increment_decrement(self):
        """Stop the repeat action"""
        if self.repeat_event:
            self.repeat_event.cancel()
            self.repeat_event = None
        self.repeat_slider_index = None
        self.repeat_direction = None
        self.repeat_count = 0

    def _repeat_action(self, dt):
        """Internal method to handle repeated increment/decrement with acceleration"""
        if self.repeat_slider_index is None or self.repeat_direction is None:
            return False
        
        # Perform the action
        if self.repeat_direction == 1:
            self.increment_parameter(self.repeat_slider_index)
        else:
            self.decrement_parameter(self.repeat_slider_index)
        
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
