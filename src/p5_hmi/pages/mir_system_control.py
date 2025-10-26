from kivymd.uix.floatlayout import MDFloatLayout
from kivy.clock import Clock

class MirSystemControlPage(MDFloatLayout):
    def change_arrow_image(self, direction):
        img = self.ids[f"arrow_{direction}"]
        img.source = "images/arrow_pressed.png"
        Clock.schedule_once(lambda dt: self.reset_arrow_image(direction), 0.2)

    def reset_arrow_image(self, direction):
        img = self.ids[f"arrow_{direction}"]
        img.source = "images/arrow.png"

    def move_forward(self):
        print("MiR: Move Forward")

    def move_backward(self):
        print("MiR: Move Backward")

    def move_left(self):
        print("MiR: Move Left")

    def move_right(self):
        print("MiR: Move Right")