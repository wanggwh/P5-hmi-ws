from kivymd.uix.floatlayout import MDFloatLayout
from kivy.uix.label import Label
from kivy.properties import StringProperty, NumericProperty, ListProperty
from kivymd.uix.label import MDLabel
from kivy.graphics import Color, RoundedRectangle
from kivy.metrics import dp

class DragonDrop(MDFloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # build buttons programmatically
        buttons = [
            {"id_name": "1", "text": "Func A", "center_x": 0.10, "bg_color": [0.23, 0.63, 0.92, 1]},
            {"id_name": "2", "text": "Func B", "center_x": 0.26, "bg_color": [0.11, 0.74, 0.61, 1]},
            {"id_name": "3", "text": "Func C", "center_x": 0.42, "bg_color": [0.54, 0.46, 0.98, 1]},
            {"id_name": "4", "text": "Func D", "center_x": 0.58, "bg_color": [0.98, 0.78, 0.29, 1]},
            {"id_name": "5", "text": "Func E", "center_x": 0.74, "bg_color": [0.94, 0.36, 0.36, 1]},
            {"id_name": "6", "text": "Sync",   "center_x": 0.90, "bg_color": [0.62, 0.65, 0.78, 1]},
        ]

        zones = [
            {"zone_id": "Alice", "pos_hint": {"x": 0.05, "y": 0.7}, "size_hint": {0.9,0.01}, "bg_color": [0.8, 0.9, 1, 0.3], "split_amount": 7},
        ]

        zone1 = DragonDropZone(
            zone_id=zones[0]["zone_id"],
            pos_hint=zones[0]["pos_hint"],
            size_hint=zones[0]["size_hint"],
            bg_color=zones[0]["bg_color"],
            split_amount=zones[0]["split_amount"],
        )

        self.add_widget(zone1)

        for spec in buttons:
            w = DragonDropButton(
                id_name=spec["id_name"],
                text=spec["text"],
                size_hint=(0.1, 0.1),
                pos_hint={"center_x": spec["center_x"], "top": 0.97},
                font_size=20,
                color=[0.96, 0.96, 0.98, 1],
                bg_color=spec["bg_color"] or [0.5,0.5,0.5,1],
                drop_zones=[zone1],
            )
            # set per-button background color if needed:
            # you can add a setter or pass bg_color kwarg; for demo reuse same bg
            self.add_widget(w)

class DragonDropButton(MDFloatLayout):
    id_name = StringProperty("")
    text = StringProperty("")
    font_size = NumericProperty(18)
    color = ListProperty([0.96, 0.96, 0.98, 1])
    bg_color = ListProperty([0.23, 0.63, 0.92, 1])  # default; override as needed
    drop_zones = ListProperty([])

    def __init__(self, **kwargs):
        # allow passing bg_color as kwarg
        bg = kwargs.pop("bg_color", None)
        super().__init__(**kwargs)
        if bg:
            self.bg_color = bg

        # rounded rect background in canvas.before
        with self.canvas.before:
            self._bg_color = Color(*self.bg_color)
            self._bg_rect = RoundedRectangle(pos=self.pos, size=self.size, radius=[8])

        # update rounded rect when pos/size change
        self.bind(pos=self._update_bg, size=self._update_bg, bg_color=self._update_bg_color)

        # label child
        label = MDLabel(
            text=self.text,
            halign="center",
            theme_text_color="Custom",
            text_color=self.color,
            font_style="Body1",
        )
        # make the label fill the layout and center it
        label.size_hint = (1, 1)
        label.pos_hint = {"center_x": 0.5, "center_y": 0.5}
        self.add_widget(label)

        self.drop_zones = kwargs.pop("drop_zones", [])

    def _update_bg(self, *a):
        self._bg_rect.pos = self.pos
        self._bg_rect.size = self.size

    def _update_bg_color(self, *a):
        self._bg_color.rgba = self.bg_color

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
                        #print(f"Dropped func: {self.id_name}, in zone: {zone.zone_id}, at coords: {self.pos}, zone coords: {coords}")
                        idx = self._find_placement(zone, zone.split_amount)
                        self.add_visual_in_zone(zone, idx)
                        zone.addToList(int(self.id_name), idx)
                        print(f"Zone {zone.zone_id} list now: {zone.order}")
                        break
                else:
                    print("Not dropped in any zone.")
            else:
                print("No zones assigned.")

            self.reset()
            return True
        return super().on_touch_up(touch)
    
    def _is_inside_zone(self, zone):
        """Check if any part of button is inside a zone."""
        bx = self.center_x
        _, by = self.pos
        _, bh = self.size
        zx, zy = zone.pos
        zw, zh = zone.size
        if bx < zx or bx > (zx + zw):
            return False
        if (by + bh) < zy or by > (zy + zh):
            return False
        return True
    
    def _find_placement(self, zone, split_amount):
        zx = zone.pos[0]
        zw = zone.size[0]
        zone_width = zw / split_amount
        idx = int((self.center_x - zx) / zone_width) + 1
        return idx

    def add_visual_in_zone(self, zone, idx):
        parent = self.parent

        # Place label in the middle of the zone segment
        center_x_hint = (zone.pos[0] + (idx - 0.5) * (zone.size[0] / zone.split_amount)) / parent.width
        # Place label at the vertical center of the zone
        center_y_hint = (zone.pos[1] + zone.size[1] / 2) / parent.height

        # Finding zone segment width
        zone_width_hint = zone.size[0] / zone.split_amount / parent.width

        # Create an MDLabel to represent the dropped button
        label = VisualCue(
            #text=self.text,
            halign="center",
            theme_text_color="Custom",
            text_color=self.color,
            font_style="Body1",
            size_hint=(zone_width_hint, 0.2),     # small width, fixed height
            #height=dp(24),
            pos_hint={"center_x": center_x_hint, "center_y": center_y_hint},
            drop_zones=[zone],
            idx=idx,
        )

        # Optional: give it the same background tint as the button
        with label.canvas.before:
            Color(*self.bg_color)
            label._bg_rect = RoundedRectangle(
                pos=label.pos,
                size=(parent.width * 0.1, dp(24)),
                radius=[8]
            )

        # Keep background in sync with label’s position
        def _update_bg_rect(instance, value):
            label._bg_rect.pos = instance.pos
            label._bg_rect.size = (instance.width, instance.height)

        label.bind(pos=_update_bg_rect, size=_update_bg_rect)

        parent.add_widget(label)

    def reset(self):
        self.pos_hint = self.startpos

class DragonDropZone(DragonDropButton):
    zone_id = StringProperty("")
    split_amount = NumericProperty(None)
    order = dict()

    #Make it not draggable
    def on_touch_down(self, touch):
        return False

    def getCoords(self):
        return self.pos_hint
    
    def addToList(self, val, pos):
        self.order[pos] = val

    def remove_from_list(self, pos):
        if pos in self.order:
            del self.order[pos]

class VisualCue(MDLabel):
    drop_zones = ListProperty([])
    idx = NumericProperty(None)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.drop_zones = kwargs.pop("drop_zones", [])
        self.idx = kwargs.get("idx")
        for zone in self.drop_zones:
            print(f'own placement: {self.idx}')
            print(f'skibidi {zone.order.get(self.idx)}')
            if zone.order.get(self.idx) is not None:
                # Remove the one that is already there
                self.parent.remove_widget(self)

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            if self.parent:
                if self.drop_zones:
                    for zone in self.drop_zones:
                        zone.remove_from_list(self.idx)
                        print(f"Zone {zone.zone_id} list now: {zone.order}")
                else: 
                    print("No zones assigned.")
                self.parent.remove_widget(self)
