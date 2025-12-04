from kivy.metrics import dp
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.anchorlayout import AnchorLayout
from kivymd.uix.button import MDRaisedButton
from kivymd.uix.card import MDCard
from kivymd.uix.label import MDLabel
from kivy.app import App


class MirSystemControlPage(BoxLayout):

    def __init__(self, **kwargs):
        super().__init__(
            orientation="vertical",
            spacing=dp(20),
            padding=dp(20),
            **kwargs
        )

        self.node = App.get_running_app().hmi_node

        buttons_anchor = AnchorLayout(
            anchor_x="center",
            anchor_y="top",
            size_hint=(1, None),
        )

        buttons_row = BoxLayout(
            orientation="horizontal",
            spacing=dp(20),
            size_hint=(0.7, None),
            height=dp(100)
        )

        btn_linear = MDRaisedButton(
            text="Linear Move",
            md_bg_color=(0.149, 0.496, 0.279, 1.0),
            size_hint=(0.7, None),
        )
        btn_linear.bind(on_release=lambda x: self.log(
            self.node.send_mission_by_name("MoveLinear")
        ))

        btn_charge = MDRaisedButton(
            text="Charge",
            md_bg_color=(0.529, 0.421, 0.157, 1.0),
            size_hint=(0.7, None),
        )
        btn_charge.bind(on_release=lambda x: self.log(
            self.node.send_mission_by_name("ChargeMir")
        ))

        btn_status = MDRaisedButton(
            text="Status",
            md_bg_color=(0.443, 0.377, 0.804, 1.0),
            size_hint=(0.7, None),
        )
        btn_status.bind(on_release=lambda x: self.log("Status not implemented yet"))

        buttons_row.add_widget(btn_linear)
        buttons_row.add_widget(btn_charge)
        buttons_row.add_widget(btn_status)

        buttons_anchor.add_widget(buttons_row)
        self.add_widget(buttons_anchor)

        card = MDCard(
            radius=[20],
            style="elevated",
            padding=dp(20),
            size_hint=(1, None),
            height=dp(330),
            md_bg_color=(0.1, 0.15, 0.30, 1)
        )

        card_layout = BoxLayout(
            orientation="vertical",
            spacing=dp(10)
        )

        title = MDLabel(
            text="--- MiR Log Output ---",
            theme_text_color="Custom",
            text_color=(1, 1, 1, 1),
            font_size=dp(16),
            size_hint=(1, None),
            height=dp(30),
            halign="center"
        )
        card_layout.add_widget(title)

        scroll = ScrollView()
        self.log_text = MDLabel(
            text="",
            halign="left",
            valign="top",
            size_hint_y=None,
            theme_text_color="Custom",
            text_color=(1, 1, 1, 1)
        )
        self.log_text.bind(texture_size=self._update_log_height)
        scroll.add_widget(self.log_text)

        card_layout.add_widget(scroll)
        card.add_widget(card_layout)
        self.add_widget(card)

    def _update_log_height(self, instance, size):
        instance.height = size[1]

    def log(self, message: str):
        self.log_text.text += f"{message}\n"
