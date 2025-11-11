from kivymd.uix.floatlayout import MDFloatLayout

class AdmittanceControl(MDFloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.app = None
        self.bob_admittance_enabled = False  # Default state for BOB - DISABLED
        self.alice_admittance_enabled = False  # Default state for ALICE - DISABLED

    def on_parent(self, widget, parent):
        """Called when the widget is added to a parent"""
        if parent:
            from kivymd.app import MDApp
            self.app = MDApp.get_running_app()

    def on_admittance_bob_toggle(self, button, active):
        """Called when BOB segmented button state changes"""
        if active:
            if button.text == "Enable":
                self.enable_bob_admittance_control()
            elif button.text == "Disable":
                self.disable_bob_admittance_control()

    def on_admittance_alice_toggle(self, button, active):
        """Called when ALICE segmented button state changes"""
        if active:
            if button.text == "Enable":
                self.enable_alice_admittance_control()
            elif button.text == "Disable":
                self.disable_alice_admittance_control()

    def enable_bob_admittance_control(self):
        """Enable admittance control for BOB"""
        self.bob_admittance_enabled = True
        print("BOB Admittance Control ENABLED")
        if self.app and hasattr(self.app, 'hmi_node'):
            # Send ROS2 message to enable BOB admittance control
            # self.app.hmi_node.send_bob_admittance_control_request(True)
            pass

    def disable_bob_admittance_control(self):
        """Disable admittance control for BOB"""
        self.bob_admittance_enabled = False
        print("BOB Admittance Control DISABLED")
        if self.app and hasattr(self.app, 'hmi_node'):
            # Send ROS2 message to disable BOB admittance control
            # self.app.hmi_node.send_bob_admittance_control_request(False)
            pass

    def enable_alice_admittance_control(self):
        """Enable admittance control for ALICE"""
        self.alice_admittance_enabled = True
        print("ALICE Admittance Control ENABLED")
        if self.app and hasattr(self.app, 'hmi_node'):
            # Send ROS2 message to enable ALICE admittance control
            # self.app.hmi_node.send_alice_admittance_control_request(True)
            pass

    def disable_alice_admittance_control(self):
        """Disable admittance control for ALICE"""
        self.alice_admittance_enabled = False
        print("ALICE Admittance Control DISABLED")
        if self.app and hasattr(self.app, 'hmi_node'):
            # Send ROS2 message to disable ALICE admittance control
            # self.app.hmi_node.send_alice_admittance_control_request(False)
            pass

    def get_bob_status(self):
        """Get current BOB admittance control status"""
        return self.bob_admittance_enabled

    def get_alice_status(self):
        """Get current ALICE admittance control status"""
        return self.alice_admittance_enabled
