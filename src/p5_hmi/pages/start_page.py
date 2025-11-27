from kivymd.uix.boxlayout import MDBoxLayout
from kivymd.uix.label import MDLabel
from kivymd.uix.list import OneLineListItem
from kivy.metrics import dp
from kivy.clock import Clock
from datetime import datetime
import os

# ...existing code...
from kivymd.uix.dialog import MDDialog
from kivymd.uix.button import MDRaisedButton

class StartPage(MDBoxLayout):
    """Start page with system control buttons and status"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Reference til main app får vi ved on_parent
        self.app = None
        self.error_messages = []  # Store error messages chronologically
        print("StartPage: __init__ called")
        self._browse_button_added = False  # ensure button only added once
        
    def on_parent(self, widget, parent):
        """Called when widget is added to parent - get app reference"""
        if parent:
            from kivymd.app import MDApp
            self.app = MDApp.get_running_app()
            print(f"StartPage: Got app reference: {self.app}")
            # Register this page to receive error messages
            if hasattr(self.app, 'hmi_node'):
                self.app.hmi_node.start_page_widget = self
                print("StartPage: Registered with HMI node for error messages")
            else:
                print("StartPage: HMI node not found yet")
        print(f"StartPage: Available IDs: {list(self.ids.keys())}")
    
    def _open_saved_configs_browser(self, *args):
        """Open a simple file browser dialog that lists files in saved_configs/"""
        save_dir = os.path.join(os.getcwd(), "saved_configs")
        if not os.path.isdir(save_dir):
            dlg = MDDialog(title="No saved_configs directory",
                           text=f"No directory found at: {save_dir}",
                           size_hint=(0.8, 0.3),
                           buttons=[])
            dlg.open()
            return

        files = sorted([f for f in os.listdir(save_dir) if os.path.isfile(os.path.join(save_dir, f))])
        if not files:
            dlg = MDDialog(title="No files",
                           text=f"No files found in: {save_dir}",
                           size_hint=(0.8, 0.3),
                           buttons=[])
            dlg.open()
            return

        content = MDBoxLayout(orientation="vertical", spacing=dp(8),
                              size_hint_y=None, height=max(len(files) * dp(48), dp(120)))
        for fname in files:
            item = OneLineListItem(text=fname,
                                   on_release=lambda inst, f=fname: self._on_saved_config_selected(f, save_dir, picker_dialog))
            content.add_widget(item)

        picker_dialog = MDDialog(title="Select saved config",
                                 type="custom",
                                 content_cls=content,
                                 size_hint=(0.9, 0.7))
        picker_dialog.open()

    def _on_saved_config_selected(self, filename, save_dir, dialog):
        """Handle selection of a saved config file"""
        try:
            dialog.dismiss()
        except Exception:
            pass

        full_path = os.path.join(save_dir, filename)
        # For now, just show a confirmation dialog and print path to console.
        print(f"StartPage: Selected saved config: {full_path}")
        confirm = MDDialog(title="Selected file",
                           text=f"{filename}\n\nPath:\n{full_path}",
                           size_hint=(0.9, 0.5),
                           buttons=[])
        confirm.open()
    
    def add_error_message(self, severity, message, node_name):
        """Add error message to chronological list"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        error_text = f"[{timestamp}] {severity} from {node_name}: {message}"
        
        print(f"StartPage: Adding error message: {error_text}")
        
        # Add to our list
        self.error_messages.append(error_text)
        
        # Keep only last 10 messages
        if len(self.error_messages) > 10:
            self.error_messages = self.error_messages[-10:]
        
        # Update UI
        self.update_error_list()
    
    def update_error_list(self):
        """Update the error list in the UI"""
        print(f"StartPage: Updating error list. Available IDs: {list(self.ids.keys())}")
        
        if hasattr(self.ids, 'error_list') and self.ids.error_list:
            print("StartPage: Found error_list widget")
            # Clear existing items
            self.ids.error_list.clear_widgets()
            
            # Add all messages (newest first)
            for error_msg in reversed(self.error_messages):
                item = OneLineListItem(
                    text=error_msg,
                    theme_text_color="Custom",
                    text_color=(0.96, 0.96, 0.98, 1),  # text_light color
                    font_style="Caption"
                )
                self.ids.error_list.add_widget(item)
                print(f"StartPage: Added error item: {error_msg}")
        else:
            print("StartPage: error_list widget not found!")
    
    def clear_error_messages(self):
        """Clear all error messages from the list"""
        print("StartPage: Clearing all error messages")
        self.error_messages.clear()
        
        # Clear the UI list
        if hasattr(self.ids, 'error_list') and self.ids.error_list:
            self.ids.error_list.clear_widgets()
            print("StartPage: Error list cleared from UI")
    
    
 