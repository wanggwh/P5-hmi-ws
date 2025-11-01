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
    
    def show_status(self, success, message, title="Status", auto_dismiss_time=2):
        """Show status popup with message"""
        status_text = "Success" if success else "Error"
        
        self.ids.status_title.text = title
        self.ids.status_message.text = f"Status: {status_text}\n{message}"
        self.open()

        if success and auto_dismiss_time > 0:
            Clock.schedule_once(lambda dt: self.dismiss(), auto_dismiss_time)