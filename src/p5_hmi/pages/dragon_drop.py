#from kivy.uix.label import Label
#from kivy.uix.behaviors import DragBehavior
from kivymd.uix.floatlayout import MDFloatLayout
from kivy.uix.label import Label
from kivy.properties import StringProperty, NumericProperty, ListProperty
from kivymd.uix.label import MDLabel
from kivy.graphics import Color, RoundedRectangle
#from kivy.metrics import dp --- IGNORE ---

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
    id_name = StringProperty("")
    zone_id = StringProperty("")
    drop_zones = ListProperty([])
    def on_touch_down(self, touch):
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
            #print(f"Dropped id:{self.id_name}, at pos {self.pos}")

            # check which zone the button was dropped into
            if self.drop_zones:
                for zone in self.drop_zones:
                    if self._is_inside_zone(zone):
                        coords = zone.getCoords()
                        print(f"Dropped func: {self.id_name}, in zone: {zone.zone_id}, at coords: {self.pos}, zone coords: {coords}")
                        break
                else:
                    print("Not dropped in any zone.")
            else:
                print("No zones assigned.")

            self.reset()
            return True
        return super().on_touch_up(touch)
    
    def _is_inside_zone(self, zone):
        """Check if button center is inside a zone."""
        bx, by = self.center
        zx, zy = zone.pos
        zw, zh = zone.size
        return zx <= bx <= zx + zw and zy <= by <= zy + zh
    
    def reset(self):
        self.pos_hint = self.startpos

class DragonDropZone(MDFloatLayout):
    def getCoords(self):
        print("Did it")
        return self.pos_hint