#!/usr/bin/env python3

from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
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

from p5_interfaces.srv import MoveToPreDefPose 

# Import page classes
from pages.start_page import StartPage

from pages.bob_configuration_setup import BobConfigurationSetup
from pages.alice_configuration_setup import AliceConfigurationSetup

from pages.mir_system_control import MirSystemControlPage
from pages.admittance_control import AdmittanceControl
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
        self.waiting_popup = None  # Single instance for waiting popup

        self.move_to_pre_def_pose_client = False

        # Subscribers
        self.error_subscriber = self.create_subscription(Error, '/error_messages', self.handle_error_message_callback, 10)
        self.status_subscriber = self.create_subscription(Bool, '/joint_mover_status', self.handle_status_message_callback, 10)
        self.joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.handle_joint_states_callback, 10)
       
        # Clients
        self.move_to_pre_def_pose_client = self.create_client(MoveToPreDefPose, "/p5_move_to_pre_def_pose")
        # self.bob_set_admittance_client = self.create_client(AdmittanceSetStatus, )

        self.get_logger().info('HMI Node has been started')
    
    def set_app(self, app):
        self.app = app

    
    def send_move_to_pre_def_pose_request(self, robot_name, goal_name):
        request = MoveToPreDefPose.Request()
        request.robot_name = robot_name
        request.goal_name = goal_name
        print(f"Preparing to send move_to_pre_def_pose request for {robot_name} to {goal_name}")


        if not self.move_to_pre_def_pose_client.wait_for_service(timeout_sec=0.1):
            if self.waiting_popup is None:
                self.waiting_popup = StatusPopupDialog()
                Clock.schedule_once(lambda dt: self.waiting_popup.waiting_on_service_popup(service_name="/p5_move_to_pre_def_pose"), 0)
            # Start periodic check for service availability
            self._pending_service_request = (request, robot_name, goal_name)
            self._service_check_event = Clock.schedule_interval(self._check_service_available_and_send, 0.5)
            return  # Exit, will continue when service is available

        # Service is available, send request immediately
        self._send_pre_def_pose_request(request, robot_name, goal_name)

    def _check_service_available_and_send(self, dt):
        if self.move_to_pre_def_pose_client.wait_for_service(timeout_sec=0.1):
            # Service is now available
            if self.waiting_popup:
                self.waiting_popup.dismiss()
                self.waiting_popup = None
            if hasattr(self, '_service_check_event'):
                self._service_check_event.cancel()
                del self._service_check_event
            if hasattr(self, '_pending_service_request'):
                request, robot_name, goal_name = self._pending_service_request
                self._send_pre_def_pose_request(request, robot_name, goal_name)
                del self._pending_service_request

    def _send_pre_def_pose_request(self, request, robot_name, goal_name):
        future = self.move_to_pre_def_pose_client.call_async(request)
        print("Service call sent, adding callback")
        # Store both robot_name and goal_name in future for use in callback
        future.robot_name = robot_name
        future.goal_name = goal_name
        future.add_done_callback(self.handle_move_to_pre_def_pose_response_callback)

    
    def handle_move_to_pre_def_pose_response_callback(self, future):
        print("Handling move_to_pre_def_pose_response")
        try:
            response = future.result()
            success = response.success
            message = response.message

            robot_name = getattr(future, "robot_name", None)
            goal_name = getattr(future, "goal_name", None)
            # Schedule GUI update in main thread
            Clock.schedule_once(lambda dt: self.app.show_status_popup(robot_name, goal_name, success, message, move_to_pre_def_pose_complete = False), 0)
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

    def handle_status_message_callback(self, msg):
        try:
            status = msg.data
            print(f"Received joint mover status: {status}")

            self.move_to_pre_def_pose_complete = msg.data

           # Clock.schedule_once(lambda dt: self.status_popup.get_status_of_request(status), 0)
        except Exception as e:
            self.get_logger().error(f"Failed to handle status message: {e}")

    def handle_joint_states_callback(self, msg):
        try:
            joint_positions = msg.position
            print(f"Received joint states: {joint_positions}")

        except Exception as e:
            self.get_logger().error(f"Failed to handle joint states message: {e}")


class HMIApp(MDApp):
    def __init__(self, ros_node, **kwargs):
        super().__init__(**kwargs)
        self.hmi_node = ros_node  
        self.colors = COLORS  # Make colors available to KV file
        self.current_page = "Start Page"  # Default page
        self.original_content = None  # Store original KV content

    def build(self):
        Window.size = (800, 480)
        Clock.schedule_interval(self.update_clock, 1)
        
        # Load main KV file 
        kv_file_path = os.path.join(os.path.dirname(__file__), 'kv/main.kv')
        root_widget = Builder.load_file(kv_file_path)
        
        # Define menu items
        menu_texts = [
            "Start Page",
            "BOB - Configuration Setup",
            "ALICE - Configuration Setup",
            "MIR - System Control",
            "Admittance Control",
            "System Logging", 
            "Settings"
        ]
        
        menu_items = [
            {
                "viewclass": "OneLineListItem",
                "text": menu_texts[i],
                "height": dp(56),
                "theme_text_color": "Custom",
                "text_color": (0, 0, 0, 1), 
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
            "BOB - Configuration Setup": "kv/bob_configuration_setup.kv",
            "ALICE - Configuration Setup": "kv/alice_configuration_setup.kv",
            "MIR - System Control": "kv/mir_system_control.kv",
            "Admittance Control": "kv/admittance_control.kv",
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
        pages = ["Start Page", "BOB - Configuration Setup", "ALICE - Configuration Setup", "MIR - System Control", "Admittance Control", "System Logging", "Settings"]
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
            elif self.current_page == "BOB - Configuration Setup":
                print("Creating BobConfigurationSetup widget")
                self.current_page_widget = BobConfigurationSetup()
                content.add_widget(self.current_page_widget)
            elif self.current_page == "ALICE - Configuration Setup":
                print("Creating BobConfigurationSetup widget")
                self.current_page_widget = AliceConfigurationSetup()
                content.add_widget(self.current_page_widget)
            elif self.current_page == "MIR - System Control":
                print("Creating MirSystemControlPage widget")
                self.current_page_widget = MirSystemControlPage()
                content.add_widget(self.current_page_widget)
            elif self.current_page == "Admittance Control":
                print("Creating AdmittanceControl widget")
                self.current_page_widget = AdmittanceControl()
                content.add_widget(self.current_page_widget)
            elif self.current_page == "System Logging":
                print("Creating SystemLoggingPage widget")
                self.current_page_widget = SystemLoggingPage()
                content.add_widget(self.current_page_widget)
            elif self.current_page == "Settings":
                print("Creating SettingsPage widget")
                self.current_page_widget = SettingsPage(self.hmi_node)
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

    def show_status_popup(self, robot_name, goal_name, success, message, move_to_pre_def_pose_complete):
        """Show a popup dialog with status message"""
        dialog = StatusPopupDialog()
        dialog.show_status(robot_name, goal_name, success, message, move_to_pre_def_pose_complete)

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