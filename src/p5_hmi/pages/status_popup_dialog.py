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
        self._is_dismissed = False
    
    @classmethod
    def create_new_dialog(cls):
        """Factory method to create a new dialog instance"""
        return cls()
    
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
    
    def show_status(self, robot_name, goal_name, success, move_to_pre_def_pose_complete=False, save_pre_def_pose_complete=False):
        """Show status popup with message and color-coded status bar"""
        # Don't show if already dismissed
        if self._is_dismissed:
            return
            
        # Reset dialog state first
        self.stop_pulsing_animation()
        
        title = f"STATUS UPDATE DIALOG"
        subtitle = "Robot operation updates"  # Default subtitle
        status_text = ""
        status_case = -1
        robot_name_corrected = robot_name

        if robot_name == "bob":
            robot_name_corrected = "BOB"
        elif robot_name == "alice":
            robot_name_corrected = "ALICE"

        if success is False:
            status_text = "The request failed, possibly due to wrong robot name or configuration"
            status_case = 0

        elif success is True and move_to_pre_def_pose_complete is True:
            status_text = "Request sent successfully, beginning operation"
            status_case = 1
        
        elif success is True and move_to_pre_def_pose_complete is False:
            status_text = "The operation completed successfully"
            status_case = 2

        self.ids.status_title.text = title
        self.ids.status_subtitle.text = subtitle
        self.ids.robot_name.text = f"Robot requested: {robot_name_corrected}"
        self.ids.goal_name.text = f"Goal requested: {goal_name}"
        self.ids.status_message.text = status_text
        
        # Open dialog first, then animate
        self.open()
        
        match status_case:
            case 0: # Error case
                target_color = [0.8, 0.2, 0.2, 1.0] # Red
                Clock.schedule_once(lambda dt: self.animate_status_bar(target_color), 0.1)

            case 1: # In progress case
                target_color = [0.8, 0.6, 0.2, 1.0] # Orange
                Clock.schedule_once(lambda dt: self.animate_pulsing_status_bar(target_color), 0.1)
               
            case 2: # Success case
                target_color = [0.2, 0.8, 0.2, 1.0] # Green
                Clock.schedule_once(lambda dt: self.animate_status_bar(target_color), 0.1)

    def show_status_save_custom_config(self, robot_name, goal_name, success):
        # Don't show if already dismissed
        if self._is_dismissed:
            return
            
        # Reset dialog state first
        self.stop_pulsing_animation()
        
        title = f"STATUS UPDATE DIALOG"
        subtitle = "Robot operation updates"  # Default subtitle
        status_text = ""
        status_case = -1
        robot_name_corrected = robot_name

        if robot_name == "bob":
            robot_name_corrected = "BOB"
        elif robot_name == "alice":
            robot_name_corrected = "ALICE"

        if success is False:
            status_text = "The request failed, possibly due to wrong robot name or configuration"
            status_case = 0

        elif success is True:
            status_text = "The configuration was stored and send correctly"
            status_case = 1
        


        self.ids.status_title.text = title
        self.ids.status_subtitle.text = subtitle
        self.ids.robot_name.text = f"Robot requested: {robot_name_corrected}"
        self.ids.goal_name.text = f"Goal requested: {goal_name}"
        self.ids.status_message.text = status_text
        
        # Open dialog first, then animate
        self.open()
        
        match status_case:
            case 0: # Error case
                target_color = [0.8, 0.2, 0.2, 1.0] # Red
                Clock.schedule_once(lambda dt: self.animate_status_bar(target_color), 0.1)

            case 1: # Success case
                target_color = [0.2, 0.8, 0.2, 1.0] # Green
                Clock.schedule_once(lambda dt: self.animate_status_bar(target_color), 0.1)
        
    
    def show_in_progress(self, configuration, message="Operation in progress..."):
        """Show in-progress status with orange status bar"""
        title = "OPERATION IN PROGRESS"
        subtitle = "Robot Operation Update"  # Default subtitle
        
        if configuration == "HOME_BOB":
            subtitle = "Homing of BOB robot arm in progress"
        elif configuration == "HOME":
            subtitle = "Homing of both robot arms in progress"
        elif configuration == "HOME_ALICE":
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
        """Override dismiss to stop animations and mark as dismissed"""
        self.stop_pulsing_animation()
        self._is_dismissed = True
        super().dismiss()

    def waiting_on_service_popup(self, service_name):
        """Show a waiting popup while waiting for a service to become available"""
        print("Showing waiting for service popup")
        
        # Don't show if already dismissed
        if self._is_dismissed:
            return
            
        # Reset dialog state first
        self.stop_pulsing_animation()
        
        title = "WAITING FOR SERVICE"
        subtitle = f"Waiting for {service_name} service to become available..."
        message = "Please wait while the system establishes connection."

        # Update content first
        self.ids.status_title.text = title
        self.ids.status_subtitle.text = subtitle
        self.ids.robot_name.text = message

        # Open dialog first
        self.open()

        # Set title bar to blue for waiting and start animation
        target_color = [0.2, 0.4, 0.8, 1.0]  # Blue
        Clock.schedule_once(lambda dt: self.animate_pulsing_status_bar(target_color), 0.1)
