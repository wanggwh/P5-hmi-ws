from kivymd.uix.boxlayout import MDBoxLayout
from kivymd.uix.label import MDLabel
from kivymd.uix.list import OneLineListItem
from kivy.metrics import dp
from kivy.clock import Clock
from datetime import datetime


class StartPage(MDBoxLayout):
    """Start page with system control buttons and status"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Reference til main app får vi ved on_parent
        self.app = None
        self.error_messages = []  # Store error messages chronologically
        print("StartPage: __init__ called")
        
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
    
    
 