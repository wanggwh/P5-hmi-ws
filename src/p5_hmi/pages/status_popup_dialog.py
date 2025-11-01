from kivymd.uix.dialog import MDDialog
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.animation import Animation
import os


class StatusPopupDialog(MDDialog):
    def __init__(self, **kwargs):
        # Load KV file if not already loaded
        kv_path = os.path.join(os.path.dirname(__file__), '..', 'kv', 'status_popup_dialog.kv')
        if not hasattr(StatusPopupDialog, '_kv_loaded'):
            Builder.load_file(kv_path)
            StatusPopupDialog._kv_loaded = True
        
        super().__init__(**kwargs)
    
    def animate_status_bar(self, target_color, duration=0.3):
        """Animate status bar color with fade effect"""
        if hasattr(self, 'ids') and 'status_title_bar' in self.ids:
            status_bar = self.ids.status_title_bar
            
            # First fade to transparent
            fade_out = Animation(
                md_bg_color=[target_color[0], target_color[1], target_color[2], 0.0],
                duration=duration/2,
                transition='in_out_cubic'
            )
            
            # Then fade in with new color
            fade_in = Animation(
                md_bg_color=target_color,
                duration=duration/2,
                transition='in_out_cubic'
            )
            
            # Chain the animations
            fade_out.bind(on_complete=lambda *args: fade_in.start(status_bar))
            fade_out.start(status_bar)
    
    def animate_pulsing_status_bar(self, target_color, pulse_duration=2.0):
        """Animate status bar with continuous pulsing fade in/out effect"""
        if hasattr(self, 'ids') and 'status_title_bar' in self.ids:
            status_bar = self.ids.status_title_bar
            
            # Create pulsing animation (fade out to transparent)
            fade_out = Animation(
                md_bg_color=[target_color[0], target_color[1], target_color[2], 0.2],
                duration=pulse_duration/2,
                transition='in_out_sine'
            )
            
            # Fade back in to full color
            fade_in = Animation(
                md_bg_color=target_color,
                duration=pulse_duration/2,
                transition='in_out_sine'
            )
            
            # Create repeating sequence
            pulse_sequence = fade_out + fade_in
            pulse_sequence.repeat = True  # Repeat forever
            
            pulse_sequence.start(status_bar)
            
            # Store reference to stop later if needed
            status_bar._pulse_animation = pulse_sequence
    
    def show_status(self, configuration, success, message, auto_dismiss_time=3):
        """Show status popup with message and color-coded status bar"""
        # Check for explicit robot failure
        if message and str(message).strip().upper() == "FAILED":
            status_text = "Error, did not complete request"
            target_color = [0.8, 0.2, 0.2, 1.0]  # Red
            error_case = True
        else:
            status_text = "Completed Request" if success else "In Progress..."
            target_color = [0.2, 0.8, 0.2, 1.0] if success else [1.0, 0.6, 0.2, 1.0]
            error_case = False
        title = "STATUS UPDATE DIALOG"
        subtitle = "Robot Operation Update"  # Default subtitle


        if message and "FAILED" in message.upper():
            subtitle = "Operation failed to complete"
        elif success is True:
            subtitle = "Operation completed successfully"
        elif configuration == "HOME_BOB":
            subtitle = "Homing of BOB robot arm initiated..."
        elif configuration == "HOME":
            subtitle = "Homing of both robot arms initiated..."
        elif configuration == "HOME_ALICES":
            subtitle = "Homing of ALICE robot arm initiated..."
        # Add more configurations as needed

        if not message:
            message_display = "No message received yet"
        else:
            message_display = message
        message_text = f"Status: {status_text}\nCommand: {configuration}\nMessage received from robots: {message_display}"

        self.ids.status_title.text = title
        self.ids.status_subtitle.text = subtitle
        self.ids.status_message.text = message_text
        self.open()

        # Animation logic
        if error_case:
            Clock.schedule_once(lambda dt: self.animate_status_bar(target_color), 0.1)
        elif success:
            Clock.schedule_once(lambda dt: self.animate_status_bar(target_color), 0.1)
        else:
            Clock.schedule_once(lambda dt: self.animate_pulsing_status_bar(target_color), 0.1)

        if success and auto_dismiss_time > 0:
            Clock.schedule_once(lambda dt: self.dismiss(), auto_dismiss_time)
    
    def show_in_progress(self, configuration, message="Operation in progress..."):
        """Show in-progress status with orange status bar"""
        title = "OPERATION IN PROGRESS"
        subtitle = "Robot Operation Update"  # Default subtitle
        
        if configuration == "HOME_BOB":
            subtitle = "Homing of BOB robot arm in progress"
        elif configuration == "HOME":
            subtitle = "Homing of both robot arms in progress"
        elif configuration == "HOME_ALICES":
            subtitle = "Homing of ALICE robot arm in progress"
        # Add more configurations as needed
        
        # Set title bar to orange for in-progress with pulsing animation
        target_color = [1.0, 0.6, 0.2, 1.0]  # Orange
        
        self.ids.status_title.text = title
        self.ids.status_subtitle.text = subtitle
        self.ids.status_message.text = f"Status: In Progress\nCommand: {configuration}"
        self.open()
        
        # Start pulsing animation after dialog opens
        Clock.schedule_once(lambda dt: self.animate_pulsing_status_bar(target_color), 0.1)
    
    def stop_pulsing_animation(self):
        """Stop the pulsing animation"""
        if hasattr(self, 'ids') and 'status_title_bar' in self.ids:
            status_bar = self.ids.status_title_bar
            if hasattr(status_bar, '_pulse_animation'):
                status_bar._pulse_animation.stop(status_bar)
    
    def dismiss(self):
        """Override dismiss to stop animations"""
        self.stop_pulsing_animation()
        super().dismiss()