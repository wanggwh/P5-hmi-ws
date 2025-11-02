from kivy.lang import Builder
from kivymd.app import MDApp
from kivymd.uix.snackbar import MDSnackbar, MDSnackbarActionButton
from kivymd.uix.label import MDLabel
from kivy.metrics import dp

KV = '''
MDScreen:
    MDRaisedButton:
        text: "Create MDSnackbar"
        on_release: app.show_md_snackbar(app.severity, app.message, app.node_name)
        pos_hint: {"center_x": .5, "center_y": .5}
'''


class ErrorMsgSnackbar(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.severity = "FATAL"
        self.message = "virker du??."
        self.node_name = "TestNode"

    def show_test_snackbar(self):
        severity = "FATAL"
        message = "virker du??."
        node_name = "TestNode"
        self.show_md_snackbar(severity, message, node_name)

    def build(self):
        return Builder.load_string(KV)

    def show_md_snackbar(self, severity, message, node_name):
        # Theme colors (should match COLORS in main app)
        bg_primary = (0.11, 0.15, 0.25, 1)      # Deep navy
        accent_coral = (0.98, 0.45, 0.32, 1)    # Coral red
        text_light = (0.96, 0.96, 0.98, 1)      # Off-white

        action_btn = MDSnackbarActionButton(
            text="Close",
            theme_text_color="Custom",
            text_color=accent_coral,
        )
        snackbar = MDSnackbar(
            MDLabel(
                text=f"[{severity}] from {node_name}: {message}",
                theme_text_color="Custom",
                text_color=text_light,
            ),
            action_btn,
            y=dp(24),
            pos_hint={"center_x": 0.5},
            size_hint_x=0.8,
            md_bg_color=bg_primary,
        )
        action_btn.on_release = lambda x=None: snackbar.dismiss()
        snackbar.open()
        snackbar.duration = 100
