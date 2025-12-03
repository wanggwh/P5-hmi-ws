from kivymd.uix.floatlayout import MDFloatLayout
from kivymd.uix.slider import MDSlider
from kivymd.uix.button import MDIconButton
from kivymd.uix.dialog import MDDialog
from kivymd.uix.boxlayout import MDBoxLayout
from kivymd.uix.button import MDFlatButton
from kivymd.uix.textfield import MDTextField
from kivy.metrics import dp
from kivy.clock import Clock
import math
from kivymd.app import MDApp
import json
import os
from datetime import datetime

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
        self.app = MDApp.get_running_app()
        self.increment_event = None
        self.publish_debounce_event = None
        
        # Track which robot is selected for tuning
        self.tuning_robot = None
        
        # Create save directory if it doesn't exist
        self.save_directory = os.path.expanduser("admittance_parameters/")
        os.makedirs(self.save_directory, exist_ok=True)
        
        saveAndParse = MDIconButton(
            icon="content-save",
            pos_hint={"center_x": 0.98, "center_y": 0.05},
            theme_icon_color="Custom",
            on_release=lambda x: self.save_admittance_parameters(),
            icon_color=[0.86, 0.86, 0.88, 1],
        )
        self.add_widget(saveAndParse)
    
    def save_admittance_parameters(self):
        # Make MDDialog with textfields to enter Name, description, date, author
        naming = {
            "Admittance tuning name": "",
            "Description": "",
            "Date": "",
            "Author": "",
        }
        
        content = MDBoxLayout(
            orientation="vertical",
            spacing=dp(10),
            size_hint_y=None,
            height=len(naming) * dp(70),
        )
        
        # Store textfields in a list so we can access them later
        textfields = []
        for param in naming:
            tf = MDTextField(hint_text=f"{param}")
            content.add_widget(tf)
            textfields.append(tf)
        
        naming_dialog = MDDialog(
            title="Save Admittance Parameters",
            type="custom",
            content_cls=content,
            buttons=[
                MDFlatButton(
                    text="CANCEL", 
                    on_release=lambda x: naming_dialog.dismiss()
                ),
                MDFlatButton(
                    text="OK", 
                    on_release=lambda x: self.on_ok(naming_dialog, textfields)
                ),
            ],
        )
        naming_dialog.open()

    def on_ok(self, dialog, textfields):
        """Handle OK button press - save parameters with metadata"""
        if self.tuning_robot is None:
            print("No robot selected - cannot save parameters")
            dialog.dismiss()
            return
        
        try:
            # Get values from textfields
            metadata = {
                "name": textfields[0].text or "Unnamed",
                "description": textfields[1].text or "No description",
                "date": textfields[2].text or datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "author": textfields[3].text or "Unknown",
            }
            
            # Get current slider values
            f_scalar = 500.0
            t_scalar = 10.0
            
            M_raw = self.ids.M_slider.value
            D_raw = self.ids.D_slider.value
            K_raw = self.ids.k_slider.value
            alpha_raw = self.ids.alpha_slider.value
            
            parameters = {
                "robot": self.tuning_robot,
                "raw_values": {
                    "M": M_raw,
                    "D": D_raw,
                    "K": K_raw,
                    "alpha": alpha_raw,
                },
                "scaled_values": {
                    "M": [M_raw*f_scalar, M_raw*f_scalar, M_raw*f_scalar, 
                          M_raw*t_scalar, M_raw*t_scalar, M_raw*t_scalar],
                    "D": [D_raw*f_scalar, D_raw*f_scalar, D_raw*f_scalar, 
                          D_raw*t_scalar, D_raw*t_scalar, D_raw*t_scalar],
                    "K": [K_raw*f_scalar, K_raw*f_scalar, K_raw*f_scalar, 
                          K_raw*t_scalar, K_raw*t_scalar, K_raw*t_scalar],
                },
                "scalars": {
                    "force_scalar": f_scalar,
                    "torque_scalar": t_scalar,
                }
            }
            
            # Combine metadata and parameters
            save_data = {
                "metadata": metadata,
                "parameters": parameters,
                "timestamp": datetime.now().isoformat(),
            }
            
            # Create filename
            safe_name = metadata['name'].replace(' ', '_').replace('/', '_')
            filename = f"admittance_{self.tuning_robot}_{safe_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            filepath = os.path.join(self.save_directory, filename)
            
            # Save to JSON file
            with open(filepath, 'w') as f:
                json.dump(save_data, f, indent=2)
            
            print(f"✅ Saved admittance parameters to: {filepath}")
            print(f"   Robot: {self.tuning_robot}")
            print(f"   M: {M_raw} (scaled: {M_raw*f_scalar})")
            print(f"   D: {D_raw} (scaled: {D_raw*f_scalar})")
            print(f"   K: {K_raw} (scaled: {K_raw*f_scalar})")
            print(f"   Alpha: {alpha_raw}")
            
            # Show success popup (optional)
            if self.app:
                success_dialog = MDDialog(
                    title="Success",
                    text=f"Parameters saved to:\n{filename}",
                    buttons=[
                        MDFlatButton(
                            text="OK",
                            on_release=lambda x: success_dialog.dismiss()
                        )
                    ],
                )
                success_dialog.open()
            
            dialog.dismiss()
            
        except Exception as e:
            print(f"❌ Failed to save parameters: {e}")
            
            # Show error popup
            error_dialog = MDDialog(
                title="Error",
                text=f"Failed to save parameters:\n{str(e)}",
                buttons=[
                    MDFlatButton(
                        text="OK",
                        on_release=lambda x: error_dialog.dismiss()
                    )
                ],
            )
            error_dialog.open()
            dialog.dismiss()
    
        

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
        """Enable BOB admittance control"""
        if self.app and hasattr(self.app, 'hmi_node'):
            # Kald eksisterende metode med korrekte parametre
            self.app.hmi_node.send_set_admittance_status_request(
                robot_name="bob",
                enable_admittance=[True for i in range(6)],
                update_rate=100.0  # Juster efter behov
            )
            # Opdater knap farver
            self.ids.bob_enable_btn.md_bg_color = self.app.colors['success']
            self.ids.bob_disable_btn.md_bg_color = self.app.colors['button_neutral']

    def disable_bob_admittance_control(self):
        """Disable BOB admittance control"""
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_set_admittance_status_request(
                robot_name="bob",
                enable_admittance=[False for i in range(6)],
                update_rate=100.0
            )
            # Opdater knap farver
            self.ids.bob_enable_btn.md_bg_color = self.app.colors['button_neutral']
            self.ids.bob_disable_btn.md_bg_color = self.app.colors['accent_coral']

    def enable_alice_admittance_control(self):
        """Enable ALICE admittance control"""
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_set_admittance_status_request(
                robot_name="alice",
                enable_admittance=[True for i in range(6)],
                update_rate=100.0
            )
            # Opdater knap farver
            self.ids.alice_enable_btn.md_bg_color = self.app.colors['success']
            self.ids.alice_disable_btn.md_bg_color = self.app.colors['button_neutral']

    def disable_alice_admittance_control(self):
        """Disable ALICE admittance control"""
        if self.app and hasattr(self.app, 'hmi_node'):
            self.app.hmi_node.send_set_admittance_status_request(
                robot_name="alice",
                enable_admittance=[False for i in range(6)],
                update_rate=100.0
            )
            # Opdater knap farver
            self.ids.alice_enable_btn.md_bg_color = self.app.colors['button_neutral']
            self.ids.alice_disable_btn.md_bg_color = self.app.colors['accent_coral']

    def get_bob_status(self):
        """Get current BOB admittance control status"""
        return self.bob_admittance_enabled

    def get_alice_status(self):
        """Get current ALICE admittance control status"""
        return self.alice_admittance_enabled

    # Slider control functions
    def on_joint_change(self, slider_index, value):
        """Called when any slider value changes"""
        value_float = round(float(value), 2)
        slider_names = ["M", "D", "k", "alpha"]
        slider_name = slider_names[slider_index]
        
        # Update the corresponding label immediately for instant feedback
        if hasattr(self.ids, f'{slider_name}_value'):
            getattr(self.ids, f'{slider_name}_value').text = f"{value_float}"
        
        # Debounce publishing - only publish after 0.3s of no changes
        if self.publish_debounce_event:
            self.publish_debounce_event.cancel()
        
        self.publish_debounce_event = Clock.schedule_once(
            lambda dt: self.publish_admittance_parameters(), 0.3)

    def tune_bob_admittance_control(self):
        """Select BOB for tuning"""
        self.tuning_robot = 'bob'
        # Update button colors
        self.ids.bob_tune_btn.md_bg_color = self.app.colors['success']
        self.ids.alice_tune_btn.md_bg_color = self.app.colors['button_neutral']
        print(f"Tuning BOB - robot selected: {self.tuning_robot}")

    def alice_tune_admittance_control(self):
        """Select ALICE for tuning"""
        self.tuning_robot = 'alice'
        # Update button colors
        self.ids.alice_tune_btn.md_bg_color = self.app.colors['success']
        self.ids.bob_tune_btn.md_bg_color = self.app.colors['button_neutral']
        print(f"Tuning ALICE - robot selected: {self.tuning_robot}")

    def publish_admittance_parameters(self):
        """Publish admittance control parameters to ROS2 for selected robot"""
        if not self.app or not hasattr(self.app, 'hmi_node'):
            return
        
        # Only publish if a robot is selected
        if self.tuning_robot is None:
            print("No robot selected for tuning")
            return
        
        try:
            M_value = self.ids.M_slider.value
            D_value = self.ids.D_slider.value
            k_value = self.ids.k_slider.value
            alpha_value = self.ids.alpha_slider.value
            
            f_scalar = 500.0   #force scalar
            t_scalar = 10.0    #torque scalar
            
            M_value = [M_value*f_scalar, M_value*f_scalar, M_value*f_scalar, M_value*t_scalar, M_value*t_scalar, M_value*t_scalar]
            D_value = [D_value*f_scalar, D_value*f_scalar, D_value*f_scalar, D_value*t_scalar, D_value*t_scalar, D_value*t_scalar]
            k_value = [k_value*f_scalar, k_value*f_scalar, k_value*f_scalar, k_value*t_scalar, k_value*t_scalar, k_value*t_scalar]
            
            # Send to the selected robot
            self.app.hmi_node.publish_admittance_parameters(
                self.tuning_robot, M_value, D_value, k_value, alpha_value)
            
        except Exception as e:
            print(f"Failed to publish admittance parameters: {e}")

    def increment_parameter(self, slider_index):
        """Increment parameter by step size"""
        slider_names = ["M_slider", "D_slider", "k_slider", "alpha_slider"]
        slider_name = slider_names[slider_index]
        if hasattr(self.ids, slider_name):
            slider = getattr(self.ids, slider_name)
            new_value = min(slider.value + slider.step, slider.max)
            slider.value = new_value

    def decrement_parameter(self, slider_index):
        """Decrement parameter by step size"""
        slider_names = ["M_slider", "D_slider", "k_slider", "alpha_slider"]
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
    
    def bob_tune_admittance_parameters(self, M, D, K):
        """Tune BOB admittance parameters directly"""
        if not self.app or not hasattr(self.app, 'hmi_node'):
            return
        try:
            f_scalar = 500.0   #force scalar
            t_scalar = 10.0    #torque scalar
            
            M_value = [M*f_scalar, M*f_scalar, M*f_scalar, M*t_scalar, M*t_scalar, M*t_scalar]
            D_value = [D*f_scalar, D*f_scalar, D*f_scalar, D*t_scalar, D*t_scalar, D*t_scalar]
            K_value = [K*f_scalar, K*f_scalar, K*f_scalar, K*t_scalar, K*t_scalar, K*t_scalar]
            # Send to ROS2
            self.app.hmi_node.publish_admittance_parameters("bob", M_value, D_value, K_value)
        except Exception as e:
            print(f"Failed to publish BOB admittance parameters: {e}")
