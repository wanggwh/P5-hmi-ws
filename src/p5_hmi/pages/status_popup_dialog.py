from kivymd.uix.dialog import MDDialog
from kivy.lang import Builder
from kivy.clock import Clock
import os


class StatusPopupDialog(MDDialog):
    def __init__(self, **kwargs):
        # Load KV file if not already loaded
        kv_path = os.path.join(os.path.dirname(__file__), '..', 'kv', 'status_popup_dialog.kv')
        if not hasattr(StatusPopupDialog, '_kv_loaded'):
            Builder.load_file(kv_path)
            StatusPopupDialog._kv_loaded = True
        
        super().__init__(**kwargs)
    
    def show_status(self, configuration, success, message, auto_dismiss_time=2):
        """Show status popup with message"""
        status_text = "Success" if success else "Error"
        title = "STATUS UPDATE DIALOG"
        subtitle = "Robot Operation Update"  # Default subtitle
        
        if configuration == "BOB_HOME":
            subtitle = "Homing of BOB robot arm initiated"
        elif configuration == "HOME":
            subtitle = "Homing of both robot arms initiated"
        elif configuration == "ALICE_HOME":
            subtitle = "Homing of ALICE robot arm initiated"
        # Add more configurations as needed
        
        self.ids.status_title.text = title
        self.ids.status_subtitle.text = subtitle
        self.ids.status_message.text = f"Command: {configuration}\nStatus: {status_text}\n{message}"
        self.open()

        if success and auto_dismiss_time > 0:
            Clock.schedule_once(lambda dt: self.dismiss(), auto_dismiss_time)