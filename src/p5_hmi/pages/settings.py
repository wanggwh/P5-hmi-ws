from kivymd.uix.boxlayout import MDBoxLayout
from kivy.app import App


class SettingsPage(MDBoxLayout):
    def __init__(self, hmi_node, **kwargs):
        super().__init__(**kwargs)
        self.hmi_node = hmi_node
    """Settings page with configuration options"""

    def set_dark_mode(self):
        print("Setting dark mode")
    def set_light_mode(self):
        print("Setting light mode")
    def show_test_error_snackbar(self):
        severity = "FATAL"
        message = "Jensens kode fucker det hele op."
        node_name = "JensenNode"
        self.hmi_node.error_snackbar.show_md_snackbar(severity, message, node_name)