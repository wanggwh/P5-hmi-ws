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
from kivymd.app import MDApp
from kivymd.uix.menu import MDDropdownMenu
from kivymd.uix.snackbar import Snackbar
from kivymd.uix.label import MDLabel
from kivymd.uix.boxlayout import MDBoxLayout
from kivy.core.window import Window
import threading

# Import page classes
from pages.start_page import StartPage
from pages.bob_system_control import BobSystemControlPage 
from pages.alice_system_control import AliceSystemControlPage 
from pages.mir_system_control import MirSystemControlPage
from pages.system_logging import SystemLoggingPage
from pages.settings import SettingsPage
from kivy.core.window import Window
Window.borderless = True
Window.left = 2800


COLORS = {
    'bg_primary': (0.11, 0.15, 0.25, 1),      # Deep navy
    'bg_secondary': (0.18, 0.24, 0.35, 1),    # Slate blue  
    'accent_orange': (0.95, 0.61, 0.23, 1),   # Warm orange
    'accent_coral': (0.98, 0.45, 0.32, 1),    # Coral red
    'text_light': (0.96, 0.96, 0.98, 1),      # Off-white
    'text_muted': (0.75, 0.78, 0.82, 1),      # Light gray
    'success': (0.32, 0.78, 0.55, 1),         # Sea green
}

class HMINode(Node):
    def __init__(self):
        super().__init__('hmi_node')

        # Publishers
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.robot_command_publisher = self.create_publisher(String, '/robot_commands', 10)

        # Subscribers
        self.error_subscription = self.create_subscription(Error, 'hmi_error', self.error_callback, 10)


        self.get_logger().info('HMI Node has been started')

    def error_callback(self, msg):
        self.get_logger().error(f'Error received: Severity={msg.severity}, Message="{msg.message}"')
        
        # Schedule error display on UI thread
        from kivymd.app import MDApp
        app = MDApp.get_running_app()
        if app:
            Clock.schedule_once(lambda dt: self.display_error_on_ui(app, msg), 0)
    
    def display_error_on_ui(self, app, error_msg):
        """Display error message on the UI - prioritize Start Page"""
        try:
            print(f"DEBUG: display_error_on_ui called with severity={error_msg.severity}, message={error_msg.message}")
            print(f"DEBUG: Current page = {getattr(app, 'current_page', 'None')}")
            
            # Always try to show on Start Page first, regardless of current page
            start_page_widget = None
            
            # If we're on Start Page, get current widget
            if hasattr(app, 'current_page') and app.current_page == "Start Page":
                start_page_widget = app.get_current_page_widget()
                print(f"DEBUG: Got start page widget = {start_page_widget}")
                print(f"DEBUG: Has show_ros_error = {hasattr(start_page_widget, 'show_ros_error') if start_page_widget else False}")
            
            # Display error on Start Page if available
            if start_page_widget and hasattr(start_page_widget, 'show_ros_error'):
                # Convert timestamp to readable format
                timestamp_str = f"{error_msg.stamp.sec}.{error_msg.stamp.nanosec//1000000:03d}"
                print(f"DEBUG: Calling show_ros_error with {error_msg.severity}, {error_msg.message}, {timestamp_str}")
                start_page_widget.show_ros_error(error_msg.severity, error_msg.message, timestamp_str)
            else:
                print("DEBUG: Start page widget not available or missing show_ros_error method")
            
            # If we're not on Start Page, show a snackbar notification
            if hasattr(app, 'current_page') and app.current_page != "Start Page":
                self.show_error_snackbar(app, error_msg)
                
        except Exception as e:
            print(f"DEBUG: Exception in display_error_on_ui: {e}")
            self.get_logger().error(f"Failed to display error on UI: {e}")
    
    def show_error_snackbar(self, app, error_msg):
        """Show error as snackbar notification when not on Start Page"""
        try:
            snackbar = Snackbar(
                text=f"[{error_msg.severity}] {error_msg.message}",
                snackbar_x="10dp",
                snackbar_y="10dp",
                size_hint_x=(Window.width - 20) / Window.width,
                bg_color=app.colors['accent_coral'] if error_msg.severity == "ERROR" else app.colors['accent_orange']
            )
            snackbar.open()
        except Exception as e:
            self.get_logger().error(f"Failed to show snackbar: {e}")

    
    def publish_error(self, severity, message, source="system"):
        """Publish an error message to the error topic"""
        try:
            error_msg = Error()
            error_msg.severity = severity
            error_msg.message = message
            error_msg.source = source
            error_msg.stamp = self.get_clock().now().to_msg()
            
            # Create error publisher if it doesn't exist
            if not hasattr(self, 'error_publisher'):
                self.error_publisher = self.create_publisher(Error, 'hmi_error', 10)
            
            self.error_publisher.publish(error_msg)
            self.get_logger().info(f'Published error: [{severity}] {message}')
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish error: {e}")

    def publish_joint_states(self, joint_positions, joint_names=None):
        """Publish joint states from HMI"""
        try:
            if joint_names is None:
                joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            
            joint_msg = JointState()
            joint_msg.header = Header()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = ''
            joint_msg.name = joint_names
            joint_msg.position = joint_positions
            joint_msg.velocity = [0.0] * len(joint_positions)
            joint_msg.effort = [0.0] * len(joint_positions)
            
            self.joint_publisher.publish(joint_msg)
            self.get_logger().info(f"Published joint states: {joint_positions}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish joint states: {e}")

    def publish_robot_command(self, command):
        msg = String()
        msg.data = command
        self.robot_command_publisher.publish(msg)
        self.get_logger().info(f"Published robot command: {command}")


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
        # Publish the selected item to ROS2

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
            # elif self.current_page == "UR Control":
            #     self.current_page_widget = URControlPage()  # Uncomment when URControlPage is created
            #     content.add_widget(self.current_page_widget)
    
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
    hmi_app.run()
    
    # Cleanup
    hmi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()