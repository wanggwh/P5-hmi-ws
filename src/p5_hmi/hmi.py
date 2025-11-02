#!/usr/bin/env python3

from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from p5_interfaces.msg import Error
import os

from kivy.lang import Builder
from kivy.metrics import dp
from kivy.clock import Clock
from kivymd.uix.dialog import MDDialog
from kivymd.app import MDApp
from kivymd.uix.menu import MDDropdownMenu
from kivymd.uix.snackbar import Snackbar
from kivymd.uix.label import MDLabel
from kivymd.uix.boxlayout import MDBoxLayout
from kivy.core.window import Window
import threading

from p5_interfaces.srv import RobotConfigurations 

# Import page classes
from pages.start_page import StartPage
from pages.bob_system_control import BobSystemControlPage 
from pages.alice_system_control import AliceSystemControlPage 
from pages.mir_system_control import MirSystemControlPage
from pages.system_logging import SystemLoggingPage
from pages.settings import SettingsPage
from pages.status_popup_dialog import StatusPopupDialog
from pages.error_msg_popup import ErrorMsgSnackbar
from kivy.core.window import Window
#Window.borderless = True
Window.left = 2800


COLORS = {
    'bg_primary': (0.11, 0.15, 0.25, 1),      # Deep navy
    'bg_secondary': (0.18, 0.24, 0.35, 1),    # Slate blue  
    'accent_orange': (0.95, 0.61, 0.23, 1),   # Warm orange
    'accent_coral': (0.98, 0.45, 0.32, 1),    # Coral red
    'button_neutral': (0.55, 0.60, 0.68, 1),  # Neutral gray-blue
    'text_light': (0.96, 0.96, 0.98, 1),      # Off-white
    'text_muted': (0.75, 0.78, 0.82, 1),      # Light gray
    'success': (0.32, 0.78, 0.55, 1),         # Sea green
}

class HMINode(Node):
    def __init__(self):
        super().__init__('hmi_node')
        self.error_snackbar = ErrorMsgSnackbar()

        self.app = None  # Reference to the Kivy app

        # Subscribers
        self.error_subscriber = self.create_subscription(Error, '/error_messages', self.handle_error_message_callback, 10)
        
        # Clients
        self.robot_configurations_client = self.create_client(RobotConfigurations, "/robot_configurations")

        self.get_logger().info('HMI Node has been started')
    
    def set_app(self, app):
        self.app = app

    
    def send_robot_configuration(self, configuration):
        request = RobotConfigurations.Request()
        request.command = configuration 

        while not self.robot_configurations_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting on /robot_configurations service...')

        future = self.robot_configurations_client.call_async(request)
        print("Service call sent, adding callback")
        # Store configuration in future for use in callback
        future.configuration = configuration
        future.add_done_callback(self.handle_robot_configuration_response)

    
    def handle_robot_configuration_response(self, future):
        print("Handling robot configuration response")
        try:
            response = future.result()
            success = response.success
            message = response.message
            configuration = future.configuration  # Get stored configuration
            # Schedule GUI update in main thread
            Clock.schedule_once(lambda dt: self.app.show_status_popup(configuration, success, message), 0)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def show_md_snackbar(self, severity, message, node_name):
        self.hmi_node.error_snackbar.show_md_snackbar(severity, message, node_name)

    def handle_error_message_callback(self, msg):
        try:
            severity = msg.severity
            message = msg.message
            node_name = msg.node_name
            print(f"Received error message: [{severity}] from {node_name}: {message}")
            # Show error snackbar in main thread
            if self.app:
                Clock.schedule_once(lambda dt: self.app.show_md_snackbar(severity, message, node_name), 0)
        except Exception as e:
            self.get_logger().error(f"Failed to handle error message: {e}")



class HMIApp(MDApp):
    def __init__(self, ros_node, **kwargs):
        super().__init__(**kwargs)
        self.hmi_node = ros_node  # Rename for clarity
        self.colors = COLORS  # Make colors available to KV file
        self.current_page = "Start Page"  # Default page
        self.original_content = None  # Store original KV content

    def build(self):
        Window.size = (800, 480)
        Clock.schedule_interval(self.update_clock, 1)
        
                # Load main KV file 
        kv_file_path = os.path.join(os.path.dirname(__file__), 'kv/main.kv')
        root_widget = Builder.load_file(kv_file_path)
        
        # Define menu items for navigation
        menu_texts = [
            "Start Page",
            "BOB - System Control",
            "ALICE - System Control",
            "MIR - System Control",
            "System Logging", 
            "Settings"
        ]
        
        menu_items = [
            {
                "viewclass": "OneLineListItem",
                "text": menu_texts[i],
                "height": dp(56),
                "theme_text_color": "Custom",
                "text_color": (0, 0, 0, 1),  # Sort tekst (RGBA)
                "md_bg_color": COLORS['bg_secondary'],
                "on_release": lambda x=menu_texts[i]: self.navigate_to_page(x),
             } for i in range(len(menu_texts))
        ]
        self.menu = MDDropdownMenu(
            items=menu_items,
            width_mult=4,
            caller=None,
            background_color=COLORS['bg_secondary'],
        )
        
        return root_widget

    def update_clock(self, dt):
        if hasattr(self.root.ids, "clock_label"):
            now = datetime.now().strftime("%H:%M:%S")
            self.root.ids.clock_label.text = now

    def load_page_kv(self, page_name):
        """Load KV files for different pages"""
        kv_files = {
            "Start Page": "kv/start_page.kv",
            "BOB - System Control": "kv/bob_system_control.kv",
            "ALICE - System Control": "kv/alice_system_control.kv",
            "MIR - System Control": "kv/mir_system_control.kv",
            "System Logging": "kv/system_logging.kv",
            "Settings": "kv/settings.kv"
        }
        
        if page_name in kv_files:
            kv_path = os.path.join(os.path.dirname(__file__), kv_files[page_name])
            # Force reload the KV file to pick up changes
            Builder.unload_file(kv_path)
            Builder.load_file(kv_path)
            return True
        return False

    def on_start(self):
        """Called when the app starts - load all KV files"""
        # Load all page KV files
        pages = ["Start Page", "BOB - System Control", "ALICE - System Control", "MIR - System Control", "System Logging", "Settings"]
        for page in pages:
            self.load_page_kv(page)
        
        # Wait for the next frame to ensure widgets are built
        Clock.schedule_once(self.load_start_page_content, 0.1)
    
    def load_start_page_content(self, dt):
        """Load start page as default content"""
        self.update_page_content()
    
    def update_clock(self, dt):
        if hasattr(self.root.ids, "clock_label"):
            now = datetime.now().strftime("%H:%M:%S")
            self.root.ids.clock_label.text = now
    
    def store_original_content(self, dt):
        """Store the original content widgets from the KV file"""
        if hasattr(self.root, 'ids') and 'content_area' in self.root.ids:
            content = self.root.ids.content_area
            # Store a copy of all current children
            self.original_content = list(content.children)

    def callback(self, button):
        self.menu.caller = button
        self.menu.open()

    def menu_callback(self, text_item):
        self.menu.dismiss()
        # Simple print instead of Snackbar to avoid compatibility issues
        print(f"Selected: {text_item}")

    def navigate_to_page(self, page_name):
        """Navigate to different pages"""
        self.menu.dismiss()
        self.current_page = page_name
        print(f"Navigating to: {page_name}")
        
        # Update the title in the top app bar
        if hasattr(self.root, 'ids') and 'top_app_bar' in self.root.ids:
            self.root.ids.top_app_bar.title = page_name
        
        # Update the content based on selected page
        self.update_page_content()
        

    def update_page_content(self):
        """Update the main content area based on current page"""
        print(f"Loading page: {self.current_page}")
        # Find the main content area and update it
        if hasattr(self.root, 'ids') and 'content_area' in self.root.ids:
            content = self.root.ids.content_area
            content.clear_widgets()
            
            # Load appropriate page widget
            if self.current_page == "Start Page":
                print("Creating StartPage widget")
                self.current_page_widget = StartPage()
                content.add_widget(self.current_page_widget)
                print(f"StartPage added. Children count: {len(content.children)}")
            elif self.current_page == "BOB - System Control":
                print("Creating BobSystemControlPage widget")
                self.current_page_widget = BobSystemControlPage()
                content.add_widget(self.current_page_widget)
            elif self.current_page == "ALICE - System Control":
                print("Creating AliceSystemControlPage widget")
                self.current_page_widget = AliceSystemControlPage()
                content.add_widget(self.current_page_widget)
            elif self.current_page == "MIR - System Control":
                print("Creating MirSystemControlPage widget")
                self.current_page_widget = MirSystemControlPage()
                content.add_widget(self.current_page_widget)
            elif self.current_page == "System Logging":
                print("Creating SystemLoggingPage widget")
                self.current_page_widget = SystemLoggingPage()
                content.add_widget(self.current_page_widget)
            elif self.current_page == "Settings":
                print("Creating SettingsPage widget")
                self.current_page_widget = SettingsPage()
                content.add_widget(self.current_page_widget)
    
    def get_current_page_widget(self):
        """Get the current page widget"""
        return getattr(self, 'current_page_widget', None)
    
    def system_button_callback(self, action):
        """Delegate system button callbacks to current page"""
        current_widget = self.get_current_page_widget()
        if current_widget and hasattr(current_widget, 'system_button_callback'):
            current_widget.system_button_callback(action)
        else:
            print(f"No handler for system action: {action}")

    def load_start_page(self, container):
        """Load Start Page content with control buttons"""
        # This recreates the original KV content programmatically
        # For now, we'll add a simple message and refer to original KV layout
        pass  # The original KV layout will show when no dynamic content is loaded

    def load_bob_system_control_page(self, container):
        """Load BOB System Control page content"""
        label = MDLabel(
            text="BOB - System Control Page",
            theme_text_color="Custom",
            text_color=self.colors['text_light'],
            halign="center"
        )
        container.add_widget(label)

    def load_system_logging_page(self, container):
        """Load System Logging page content"""
        label = MDLabel(
            text="System Logging Page - View logs here",
            theme_text_color="Custom",
            text_color=self.colors['text_light'],
            halign="center"
        )
        container.add_widget(label)

    def load_settings_page(self, container):
        """Load Settings page content"""
        label = MDLabel(
            text="Settings Page - Configure system here",
            theme_text_color="Custom",
            text_color=self.colors['text_light'],
            halign="center"
        )
        container.add_widget(label)

    def show_status_popup(self, configuration, success, message):
        """Show a popup dialog with status message"""
        dialog = StatusPopupDialog()
        dialog.show_status(configuration, success, message)

    def show_md_snackbar(self, severity, message, node_name):
        self.hmi_node.error_snackbar.show_md_snackbar(severity, message, node_name)


def ros_spin(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    
    # Create ROS2 node
    hmi_node = HMINode()
    
    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(hmi_node,))
    ros_thread.daemon = True
    ros_thread.start()
    
    # Create and run Kivy app
    hmi_app = HMIApp(hmi_node)
    hmi_node.set_app(hmi_app)
    hmi_app.run()
    
    # Cleanup
    hmi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()