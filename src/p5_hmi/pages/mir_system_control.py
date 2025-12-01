# mir_hmi.py
import rclpy
from rclpy.node import Node
from p5_interfaces.srv import PostMissions, GetMissions

from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivymd.uix.button import MDFlatButton

class MirHMI(Node):
    def __init__(self):
        super().__init__('mir_hmi_node')
        # Clients for ROS 2 services
        self.post_client = self.create_client(PostMissions, 'post_mission')
        self.get_client = self.create_client(GetMissions, 'get_missions')

        # Wait for service availability
        while not self.post_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /post_mission service...')
        while not self.get_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /get_missions service...')

    def post_mission(self, mission_name):
        # Request missions first to get GUID
        req = GetMissions.Request()
        future = self.get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None or not future.result().success:
            self.get_logger().error("Failed to get missions")
            return
        # Find GUID by name
        missions = dict(zip(future.result().names, future.result().guids))
        guid = missions.get(mission_name)
        if not guid:
            self.get_logger().error(f"Mission '{mission_name}' not found")
            return

        # Post mission
        req_post = PostMissions.Request()
        req_post.mission_guid = guid
        future_post = self.post_client.call_async(req_post)
        rclpy.spin_until_future_complete(self, future_post)
        if future_post.result() is not None and future_post.result().success:
            self.get_logger().info(f"Mission '{mission_name}' queued successfully!")
        else:
            msg = future_post.result().message if future_post.result() else "No response"
            self.get_logger().error(f"Failed to queue '{mission_name}': {msg}")


class MirHMIApp(App):
    def build(self):
        self.node = MirHMI()

        self.layout = BoxLayout(orientation='vertical', spacing=10, padding=10)

        # Top-level buttons
        self.start_mission_btn = MDFlatButton(text="Start Mission", size_hint_y=None, height=50)
        self.start_mission_btn.bind(on_press=self.toggle_mission_buttons)
        self.status_btn = MDFlatButton(text="Status", size_hint_y=None, height=50)
        self.status_btn.bind(on_press=self.check_status)

        self.layout.add_widget(self.start_mission_btn)
        self.layout.add_widget(self.status_btn)

        # Hidden mission buttons
        self.mission_buttons_layout = BoxLayout(orientation='vertical', spacing=5, size_hint_y=None)
        self.mission_buttons_layout.height = 0  # Start hidden

        self.charge_btn = MDFlatButton(text="Charge", size_hint_y=None, height=50)
        self.charge_btn.bind(on_press=lambda x: self.node.post_mission("ChargeMir"))

        self.start_test_btn = MDFlatButton(text="Start Test", size_hint_y=None, height=50)
        self.start_test_btn.bind(on_press=lambda x: self.node.post_mission("MoveLinear"))

        self.mission_buttons_layout.add_widget(self.charge_btn)
        self.mission_buttons_layout.add_widget(self.start_test_btn)

        self.layout.add_widget(self.mission_buttons_layout)
        return self.layout

    def toggle_mission_buttons(self, instance):
        if self.mission_buttons_layout.height == 0:
            self.mission_buttons_layout.height = 110  # Show
        else:
            self.mission_buttons_layout.height = 0  # Hide

    def check_status(self, instance):
        # Example: you can expand this to call GetMissions or other status endpoints
        print("Status button pressed")


if __name__ == '__main__':
    rclpy.init()
    MirHMIApp().run()
    rclpy.shutdown()
