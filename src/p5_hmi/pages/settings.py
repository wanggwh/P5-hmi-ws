from kivymd.uix.boxlayout import MDBoxLayout


class SettingsPage(MDBoxLayout):
    """Settings page with configuration options"""

    def set_dark_mode(self):
        print("Setting dark mode")
    def set_light_mode(self):
        print("Setting light mode")