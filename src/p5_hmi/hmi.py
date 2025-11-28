#!/usr/bin/env python3

from datetime import datetime
import rclpy
from rclpy.node import Node
import os
import math
import threading

from kivy.lang import Builder
from kivy.metrics import dp
from kivy.clock import Clock
from kivymd.app import MDApp
from kivymd.uix.menu import MDDropdownMenu
from kivy.core.window import Window

from p5_interfaces.srv import PoseConfig
from p5_interfaces.srv import MoveToPreDefPose#, SaveProgram
from p5_interfaces.srv import AdmittanceSetStatus, AdmittanceConfig
from p5_interfaces.msg import CommandState
from p5_interfaces.msg import Error
from sensor_msgs.msg import JointState


# Import page classes
from pages.start_page import StartPage
from pages.bob_configuration_setup import BobConfigurationSetup
from pages.alice_configuration_setup import AliceConfigurationSetup
from pages.mir_system_control import MirSystemControlPage
from pages.admittance_control import AdmittanceControl
from pages.dragon_drop import DragonDrop
from pages.system_logging import SystemLoggingPage
from pages.settings import SettingsPage
from pages.status_popup_dialog import StatusPopupDialog
from pages.error_msg_popup import ErrorMsgSnackbar

# Window.borderless = True
Window.left = 300

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
    """ROS2 Node for HMI communication and service handling"""
    
    def __init__(self):
        super().__init__('hmi_node')
        self.error_snackbar = ErrorMsgSnackbar()

        # App references
        self.app = None
        self.waiting_popup = None
        self.current_status_dialog = None
        self.start_page_widget = None
        self._operation_completed = False

        # Initialize robot joint positions
        self.alice = [0.0] * 6
        self.bob = [0.0] * 6

        # Throttle variables for joint states
        self._last_joint_update = 0
        self._joint_update_interval = 0.1  # Max 10 Hz opdatering til GUI

        # Subscribers
        self.error_subscriber = self.create_subscription(
            Error, '/error_messages', self.handle_error_message_callback, 10)
        self.status_subscriber = self.create_subscription(
            CommandState, '/p5_command_state', self.handle_status_message_callback, 10)
        self.joint_states_subscriber = self.create_subscription(
            JointState, '/joint_states', self.handle_joint_states_callback, 10)

        # Service clients
        self.move_to_pre_def_pose_client = self.create_client(
            MoveToPreDefPose, "/p5_move_to_pre_def_pose")
        self.save_pre_def_pose_client = self.create_client(
            PoseConfig, "/p5_pose_config")
        # self.save_program_client = self.create_client(
        #     SaveProgram, "/program_executor/save_program")

        self.get_logger().info('HMI Node has been started')

    def set_app(self, app):
        """Set reference to Kivy app"""
        self.app = app

    # ==================== Admittance Control ====================
    
    def publish_admittance_parameters(self, robot_name, M_parameter, D_parameter, K_parameter):
        client = self.create_client(
            AdmittanceConfig, "/" + robot_name + "/p5_admittance_config")
        request = AdmittanceConfig.Request()
        request.m = M_parameter
        request.d = D_parameter
        request.k = K_parameter
        request.alpha = 0.01
        
        if not client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning(
                f"AdmittanceConfig service not available, cannot send request for {robot_name}")
            self.waiting_popup = StatusPopupDialog.create_new_dialog()
            Clock.schedule_once(
                lambda dt: self.waiting_popup.waiting_on_service_popup(
                    service_name="/" + robot_name + "/p5_admittance_config"), 0)
            self._pending_service_request = (client, request, M_parameter, D_parameter, K_parameter)
            self._service_check_event = Clock.schedule_interval(
                self._check_admittance_config_service_available_and_send, 0.5)
            return
        self._send_admittance_config_request(client, request, M_parameter, D_parameter, K_parameter)
        
    def send_set_admittance_status_request(self, robot_name, enable_admittance, update_rate):
        """Send admittance control status request"""
        client = self.create_client(
            AdmittanceSetStatus, "/" + robot_name + "/p5_admittance_set_state")
        request = AdmittanceSetStatus.Request()
        request.active = enable_admittance
        request.update_rate = int(update_rate)

        if not client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning(
                f"AdmittanceSetStatus service not available, cannot send request for {robot_name}")
            self.get_logger()
            self.waiting_popup = StatusPopupDialog.create_new_dialog()
            Clock.schedule_once(
                lambda dt: self.waiting_popup.waiting_on_service_popup(
                    service_name="/" + robot_name + "/p5_admittance_set_state"), 0)
            self._pending_service_request = (client, request, enable_admittance, update_rate)
            self._service_check_event = Clock.schedule_interval(
                self._check_admittance_service_available_and_send, 0.5)
            return
        
        self._send_set_admittance_request(client, request, enable_admittance, update_rate)

    def _check_admittance_service_available_and_send(self, dt):
        """Periodically check if admittance service is available and send pending request"""
        #if self.set_admittance_status_client.wait_for_service(timeout_sec=0.1):
        if self.waiting_popup:
            self.waiting_popup.dismiss()
            self.waiting_popup = None
            
        if hasattr(self, '_service_check_event'):
            self._service_check_event.cancel()
            del self._service_check_event
            
        if hasattr(self, '_pending_service_request'):
            client, request, enable_admittance, update_rate = self._pending_service_request
            self._send_set_admittance_request(client, request, enable_admittance, update_rate)
            del self._pending_service_request
                
    def _check_admittance_config_service_available_and_send(self, dt):
        """Periodically check if admittance service is available and send pending request"""
        #if self.set_admittance_status_client.wait_for_service(timeout_sec=0.1):
        if self.waiting_popup:
            self.waiting_popup.dismiss()
            self.waiting_popup = None
            
        if hasattr(self, '_service_check_event'):
            self._service_check_event.cancel()
            del self._service_check_event
            
        if hasattr(self, '_pending_service_request'):
            client, request, M_parameter, D_parameter, K_parameter = self._pending_service_request
            self._send_admittance_config_request(client, request, M_parameter, D_parameter, K_parameter)
            del self._pending_service_request

    def _send_set_admittance_request(self, client, request, enable_admittance, update_rate):
        """Send admittance request to service"""
        self._operation_completed = False
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"Admittance status set to {enable_admittance}"))
    
    def _send_admittance_config_request(self, client, request, M_parameter, D_parameter, K_parameter):
        """Send admittance request to service"""
        self._operation_completed = False
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"Admittance parameter set to {M_parameter}"))

    # ==================== Move to Predefined Pose ====================
    
    def send_move_to_pre_def_pose_request(self, robot_name, goal_name):
        """Send request to move robot to predefined pose"""
        request = MoveToPreDefPose.Request()
        request.robot_name = robot_name
        request.goal_name = goal_name
        print(f"Preparing to send move_to_pre_def_pose request for {robot_name} to {goal_name}")

        if not self.move_to_pre_def_pose_client.wait_for_service(timeout_sec=0.1):
            self.waiting_popup = StatusPopupDialog.create_new_dialog()
            Clock.schedule_once(
                lambda dt: self.waiting_popup.waiting_on_service_popup(
                    service_name="/p5_move_to_pre_def_pose"), 0)
            self._pending_service_request = (request, robot_name, goal_name)
            self._service_check_event = Clock.schedule_interval(
                self._check_service_available_and_send, 0.5)
            return

        self._send_pre_def_pose_request(request, robot_name, goal_name)

    def _check_service_available_and_send(self, dt):
        """Periodically check if move service is available and send pending request"""
        if self.move_to_pre_def_pose_client.wait_for_service(timeout_sec=0.1):
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
        """Send predefined pose request to service"""
        # Store robot info for status callback
        self._current_robot_name = robot_name
        self._current_goal_name = goal_name
        self._operation_completed = False

        future = self.move_to_pre_def_pose_client.call_async(request)
        print("Service call sent, adding callback")
        future.robot_name = robot_name
        future.goal_name = goal_name
        future.add_done_callback(self.handle_move_to_pre_def_pose_response_callback)

    def handle_move_to_pre_def_pose_response_callback(self, future):
        """Handle response from move to predefined pose service"""
        print("Handling move_to_pre_def_pose_response")
        try:
            response = future.result()
            success = response.success
            robot_name = getattr(future, "robot_name", None)
            goal_name = getattr(future, "goal_name", None)
            print("robot_name: " + robot_name)
            def create_and_store_dialog(dt):
                self.current_status_dialog = StatusPopupDialog.create_new_dialog()
                self.current_status_dialog.show_status_dialog(
                    robot_name, goal_name, success, move_to_pre_def_pose_complete=True)

            Clock.schedule_once(create_and_store_dialog, 0)
            
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    # ==================== Save Predefined Pose ====================
    
    def save_pre_def_pose_request(self, robot_name, goal_name):
        """Send request to save current pose as predefined pose"""
        request = PoseConfig.Request()
        request.robot_name = robot_name
        request.pose = goal_name

        if not self.save_pre_def_pose_client.wait_for_service(timeout_sec=0.1):
            self.waiting_popup = StatusPopupDialog.create_new_dialog()
            Clock.schedule_once(
                lambda dt: self.waiting_popup.waiting_on_service_popup(
                    service_name="/p5_pose_config"), 0)
            self._pending_service_request = (request, robot_name, goal_name)
            self._service_check_event = Clock.schedule_interval(
                self._check_save_pose_service_available_and_send, 0.5)
            return

        self._save_pre_def_pose_request(request, robot_name, goal_name)

    def _check_save_pose_service_available_and_send(self, dt):
        """Periodically check if save pose service is available and send pending request"""
        if self.save_pre_def_pose_client.wait_for_service(timeout_sec=0.1):
            if self.waiting_popup:
                self.waiting_popup.dismiss()
                self.waiting_popup = None
            
            if hasattr(self, '_service_check_event'):
                self._service_check_event.cancel()
                del self._service_check_event
            
            if hasattr(self, '_pending_service_request'):
                request, robot_name, goal_name = self._pending_service_request
                self._save_pre_def_pose_request(request, robot_name, goal_name)
                del self._pending_service_request

    def _save_pre_def_pose_request(self, request, robot_name, goal_name):
        #Send save pose request to service
        self._current_robot_name = robot_name
        self._current_goal_name = goal_name

        future = self.save_pre_def_pose_client.call_async(request)
        print("Service call sent, adding callback")
        future.robot_name = robot_name
        future.goal_name = goal_name
        future.add_done_callback(self.handle_save_pre_def_pose_response_callback)

    def handle_save_pre_def_pose_response_callback(self, future):
        #Handle response from save predefined pose service
        print("Handling save_pre_def_pose_response")
        try:
            response = future.result()
            success = response.message
            robot_name = getattr(future, "robot_name", None)
            goal_name = getattr(future, "goal_name", None)

            def create_and_store_dialog(dt):
                self.current_status_dialog = StatusPopupDialog.create_new_dialog()
                self.current_status_dialog.show_status_save_custom_config(robot_name, goal_name, success)

            Clock.schedule_once(create_and_store_dialog, 0)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    # ==================== Save Program ====================
    
    # def call_save_program_request(self, program_name: str, program_json: str, 
    #                                wait_for_service=True, timeout=0.1) -> bool:
    #     """
    #     Send request to save program using reusable SaveProgram client.
    #     Assigns program_name to first string field and program_json to second.
    #     """
    #     if not hasattr(self, 'save_program_client') or self.save_program_client is None:
    #         self.save_program_client = self.create_client(
    #             SaveProgram, "/program_executor/save_program")

    #     client = self.save_program_client
    #     req = SaveProgram.Request()

    #     # Detect string fields from the generated Request
    #     try:
    #         fields = req.get_fields_and_field_types()
    #     except Exception:
    #         fields = {}

    #     string_fields = [n for n, t in fields.items() if 'string' in t]

    #     # Assign program_name and program_json to string fields
    #     assigned = []
    #     try:
    #         if string_fields:
    #             if len(string_fields) >= 1:
    #                 setattr(req, string_fields[0], program_name)
    #                 assigned.append(string_fields[0])
    #             if len(string_fields) >= 2:
    #                 setattr(req, string_fields[1], program_json)
    #                 assigned.append(string_fields[1])
    #         else:
    #             # Fallback: try common candidate names
    #             candidates = ['name', 'program', 'program_json', 'program_name', 
    #                          'data', 'json', 'content', 'file', 'text']
    #             targets = [program_name, program_json]
    #             for payload in targets:
    #                 for cand in candidates:
    #                     if hasattr(req, cand) and cand not in assigned:
    #                         try:
    #                             setattr(req, cand, payload)
    #                             assigned.append(cand)
    #                             break
    #                         except Exception:
    #                             continue
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to populate SaveProgram request fields: {e}")

    #     if not assigned:
    #         self.get_logger().error(
    #             "SaveProgram request: no suitable string fields found on request object")
    #         return False

    #     def _send_request():
    #         try:
    #             future = client.call_async(req)
    #             future.program_name = program_name
    #             future.add_done_callback(self._handle_save_program_response_callback)
    #             self.get_logger().info(
    #                 f"SaveProgram request sent for '{program_name}' (fields used: {assigned})")
    #         except Exception as e:
    #             self.get_logger().error(f"SaveProgram client failed to call service: {e}")

    #     # Wait for service if requested
    #     if wait_for_service and not client.wait_for_service(timeout_sec=0.1):
    #         waiting_popup = StatusPopupDialog.create_new_dialog()
    #         Clock.schedule_once(
    #             lambda dt: waiting_popup.waiting_on_service_popup(
    #                 service_name="/program_executor/save_program"), 0)

    #         check_event = {"evt": None}

    #         def _check_service(dt):
    #             if client.wait_for_service(timeout_sec=0.1):
    #                 try:
    #                     if waiting_popup:
    #                         waiting_popup.dismiss()
    #                 except Exception:
    #                     pass
    #                 _send_request()
    #                 if check_event["evt"]:
    #                     check_event["evt"].cancel()
    #                     check_event["evt"] = None

    #         check_event["evt"] = Clock.schedule_interval(_check_service, 0.5)
    #         return True

    #     try:
    #         _send_request()
    #         return True
    #     except Exception as e:
    #         self.get_logger().error(f"SaveProgram immediate call failed: {e}")
    #         return False

    # def _handle_save_program_response_callback(self, future):
    #     """Handle SaveProgram response"""
    #     try:
    #         resp = future.result()
    #         success = getattr(resp, "success", None)
    #         message = getattr(resp, "message", str(resp))
    #         pname = getattr(future, "program_name", "Unknown")
    #         self.get_logger().info(
    #             f"SaveProgram response for '{pname}': success={success}, message={message}")

    #         if self.app:
    #             Clock.schedule_once(
    #                 lambda dt: self.app.show_status_popup(
    #                     pname, "save_program", bool(success), message, 
    #                     move_to_pre_def_pose_complete=False), 0)
    #     except Exception as e:
    #         self.get_logger().error(f"SaveProgram response handler error: {e}")

    # ==================== Topic Callbacks ====================
    
    def handle_error_message_callback(self, msg):
        """Handle error messages from ROS topics"""
        try:
            severity = msg.severity
            message = msg.message
            node_name = msg.node_name
            print(f"HMI: Received error message: [{severity}] from {node_name}: {message}")

            # Show error snackbar in main thread
            if self.app:
                Clock.schedule_once(
                    lambda dt: self.app.show_md_snackbar(severity, message, node_name), 0)

            # Add to start page error list if available
            if hasattr(self, 'start_page_widget') and self.start_page_widget:
                print(f"HMI: Sending error to start page widget: {self.start_page_widget}")
                Clock.schedule_once(
                    lambda dt: self.start_page_widget.add_error_message(
                        severity, message, node_name), 0)
            else:
                print("HMI: No start page widget registered for error logging")

        except Exception as e:
            self.get_logger().error(f"Failed to handle error message: {e}")

    def handle_status_message_callback(self, msg):
        """Handle status updates from command state topic"""
        try:
            status = msg.status
            self.move_to_pre_def_pose_complete = msg.status

            # When operation starts (status=True), reset completion flag
            if status and self.app:
                self._operation_completed = False

            # When operation completes (status=False) and we haven't shown success yet
            if not status and self.app and not self._operation_completed:
                self._operation_completed = True
                
                robot_name = getattr(self, '_current_robot_name', 'Unknown')
                goal_name = getattr(self, '_current_goal_name', 'Unknown')

                def show_success_and_close_previous(dt):
                    # Close orange dialog if it exists
                    if self.current_status_dialog:
                        self.current_status_dialog.dismiss()
                        self.current_status_dialog = None

                    # Create and store new green success dialog
                    self.current_status_dialog = StatusPopupDialog.create_new_dialog()
                    self.current_status_dialog.show_status_dialog(
                        robot_name, goal_name, True, move_to_pre_def_pose_complete=False)

                Clock.schedule_once(show_success_and_close_previous, 0)

        except Exception as e:
            self.get_logger().error(f"Failed to handle status message: {e}")

    def handle_joint_states_callback(self, msg):
        """Handle joint state updates and update GUI"""
        try:
            # Throttle updates - kun opdater GUI hvert 0.1 sekund
            import time
            current_time = time.time()
            if current_time - self._last_joint_update < self._joint_update_interval:
                return
            
            self._last_joint_update = current_time
            
            # Update Alice joint positions (convert from radians to degrees)
            self.alice[0] = math.degrees(msg.position[2])
            self.alice[1] = math.degrees(msg.position[1])
            self.alice[2] = math.degrees(msg.position[0])
            self.alice[3] = math.degrees(msg.position[3])
            self.alice[4] = math.degrees(msg.position[4])
            self.alice[5] = math.degrees(msg.position[5])

            # Update Bob joint positions (convert from radians to degrees)
            self.bob[0] = math.degrees(msg.position[8])
            self.bob[1] = math.degrees(msg.position[7])
            self.bob[2] = math.degrees(msg.position[6])
            self.bob[3] = math.degrees(msg.position[9])
            self.bob[4] = math.degrees(msg.position[10])
            self.bob[5] = math.degrees(msg.position[11])
            
            if self.app:
                # Batch begge opdateringer i ét Clock.schedule_once
                def update_both(dt):
                    self.app.bob_update_joint_positions(self.bob)
                    self.app.alice_update_joint_positions(self.alice)
                
                Clock.schedule_once(update_both, 0)

        except Exception as e:
            self.get_logger().error(f"Failed to handle joint states message: {e}")

    def show_md_snackbar(self, severity, message, node_name):
        """Show error snackbar in GUI"""
        self.hmi_node.error_snackbar.show_md_snackbar(severity, message, node_name)


class HMIApp(MDApp):
    """Main Kivy application for HMI interface"""
    
    def __init__(self, ros_node, **kwargs):
        super().__init__(**kwargs)
        self.hmi_node = ros_node
        self.colors = COLORS
        self.current_page = "Start Page"
        self.original_content = None

    def build(self):
        """Build and return the root widget"""
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
            "Drag and Drop - System Control",
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
        """Update clock display in top bar"""
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
            "Drag and Drop - System Control": "kv/dragon_drop.kv",
            "System Logging": "kv/system_logging.kv",
            "Settings": "kv/settings.kv"
        }

        if page_name in kv_files:
            kv_path = os.path.join(os.path.dirname(__file__), kv_files[page_name])
            Builder.unload_file(kv_path)
            Builder.load_file(kv_path)
            return True
        return False

    def on_start(self):
        """Called when the app starts - load all KV files"""
        pages = [
            "Start Page", "BOB - Configuration Setup", "ALICE - Configuration Setup",
            "MIR - System Control", "Admittance Control", "Drag and Drop - System Control",
            "System Logging", "Settings"
        ]
        
        for page in pages:
            self.load_page_kv(page)

        Clock.schedule_once(lambda dt: self.update_page_content(), 0.1)

    def store_original_content(self, dt):
        """Store the original content widgets from the KV file"""
        if hasattr(self.root, 'ids') and 'content_area' in self.root.ids:
            content = self.root.ids.content_area
            self.original_content = list(content.children)

    def callback(self, button):
        """Open navigation menu"""
        self.menu.caller = button
        self.menu.open()

    def menu_callback(self, text_item):
        """Handle menu item selection"""
        self.menu.dismiss()
        print(f"Selected: {text_item}")

    def navigate_to_page(self, page_name):
        """Navigate to different pages"""
        self.menu.dismiss()
        self.current_page = page_name
        print(f"Navigating to: {page_name}")

        # Update title in top app bar
        if hasattr(self.root, 'ids') and 'top_app_bar' in self.root.ids:
            self.root.ids.top_app_bar.title = page_name

        self.update_page_content()

    def update_page_content(self):
        """Update the main content area based on current page"""
        print(f"Loading page: {self.current_page}")
        
        if hasattr(self.root, 'ids') and 'content_area' in self.root.ids:
            content = self.root.ids.content_area
            content.clear_widgets()

            # Load appropriate page widget
            page_widgets = {
                "Start Page": StartPage,
                "BOB - Configuration Setup": BobConfigurationSetup,
                "ALICE - Configuration Setup": AliceConfigurationSetup,
                "MIR - System Control": MirSystemControlPage,
                "Admittance Control": AdmittanceControl,
                "Drag and Drop - System Control": DragonDrop,
                "System Logging": SystemLoggingPage,
                "Settings": lambda: SettingsPage(self.hmi_node)
            }

            if self.current_page in page_widgets:
                print(f"Creating {self.current_page} widget")
                widget_class = page_widgets[self.current_page]
                self.current_page_widget = widget_class() if callable(widget_class) else widget_class()
                content.add_widget(self.current_page_widget)
                print(f"{self.current_page} added. Children count: {len(content.children)}")

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

    def show_md_snackbar(self, severity, message, node_name):
        """Show error snackbar"""
        self.hmi_node.error_snackbar.show_md_snackbar(severity, message, node_name)

    def bob_update_joint_positions(self, joint_positions):
        """Update BOB joint positions in current page widget"""
        current_widget = self.get_current_page_widget()
        if current_widget and hasattr(current_widget, 'bob_update_joint_positions'):
            current_widget.bob_update_joint_positions(joint_positions)
    
    def alice_update_joint_positions(self, joint_positions):
        """Update ALICE joint positions in current page widget"""
        current_widget = self.get_current_page_widget()
        if current_widget and hasattr(current_widget, 'alice_update_joint_positions'):
            current_widget.alice_update_joint_positions(joint_positions)
        

def ros_spin(node):
    """Spin ROS2 node in separate thread"""
    rclpy.spin(node)


def main(args=None):
    """Main entry point for HMI application"""
    rclpy.init(args=args)
    
    # Create ROS2 node
    hmi_node = HMINode()
    
    # Start ROS2 spinning in separate thread
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

