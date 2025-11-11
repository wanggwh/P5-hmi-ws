#from kivy.uix.label import Label
#from kivy.uix.behaviors import DragBehavior
from kivymd.uix.floatlayout import MDFloatLayout
from kivy.uix.label import Label
from kivy.properties import StringProperty

class DragonDrop(MDFloatLayout):
    pass


class DragonDropButton(Label):
    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            self._touch_offset_x = self.x - touch.x
            self._touch_offset_y = self.y - touch.y
            touch.grab(self)
            return True
        return super().on_touch_down(touch)

    def on_touch_move(self, touch):
        if touch.grab_current is self:
            self.x = touch.x + self._touch_offset_x
            self.y = touch.y + self._touch_offset_y
            return True
        return super().on_touch_move(touch)

    def on_touch_up(self, touch):
        if touch.grab_current is self:
            touch.ungrab(self)
            return True
        return super().on_touch_up(touch)
    
class DragonDropButtonHint(MDFloatLayout):
    def on_touch_down(self, touch):
        id_name = StringProperty("")
        self.startpos = self.pos_hint.copy()
        if self.collide_point(*touch.pos):
            if not self.parent:
                return False

            self._touch_offset_x = (self.x - touch.x) / self.parent.width
            self._touch_offset_y = (self.y - touch.y) / self.parent.height

            touch.grab(self)
            return True
        return super().on_touch_down(touch)

    def on_touch_move(self, touch):
        if touch.grab_current is self:

            rel_x = (touch.x / self.parent.width) + self._touch_offset_x
            rel_y =(touch.y /self.parent.height) + self._touch_offset_y

            # Clamp to layout bounds (0–1)
            rel_x = max(0, min(1 - self.size_hint_x if self.size_hint_x else 1, rel_x))
            rel_y = max(0, min(1 - self.size_hint_y if self.size_hint_y else 1, rel_y))

            self.pos_hint = {"x": rel_x, "y": rel_y}

            return True
        return super().on_touch_move(touch)

    def on_touch_up(self, touch):
        if touch.grab_current is self:
            touch.ungrab(self)
            print(f"Dropped id:{self.id_name}, at absolute pos {self.pos}, relative pos_hint {self.pos_hint}")
            self.reset()
            return True
        return super().on_touch_up(touch)
    
    def reset(self):
        self.pos_hint = self.startpos
