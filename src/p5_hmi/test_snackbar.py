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


class Test(MDApp):
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
        action_btn = MDSnackbarActionButton(
            text="Close",
            theme_text_color="Custom",
            text_color="#8E353C",
        )
        snackbar = MDSnackbar(
            MDLabel(
                text=f"[{severity}] from {node_name}: {message}",
            ),
            action_btn,
            y=dp(24),
            pos_hint={"center_x": 0.5},
            size_hint_x=0.8,
            md_bg_color="#E8D8D7",
        )
        action_btn.on_release = lambda x=None: snackbar.dismiss()
        snackbar.open()
        snackbar.duration = 100 

if __name__ == "__main__":
    Test().run()