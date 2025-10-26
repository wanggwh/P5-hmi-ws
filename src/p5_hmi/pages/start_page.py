from kivymd.uix.boxlayout import MDBoxLayout
from kivymd.uix.label import MDLabel
from kivy.metrics import dp
from kivy.clock import Clock



class StartPage(MDBoxLayout):
    """Start page with system control buttons and status"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Reference til main app får vi ved on_parent
        self.app = None
        print("StartPage: __init__ called")
        
    def on_parent(self, widget, parent):
        """Called when widget is added to parent - get app reference"""
        if parent:
            from kivymd.app import MDApp
            self.app = MDApp.get_running_app()
    
    
 