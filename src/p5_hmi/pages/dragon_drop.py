from copy import deepcopy
from kivymd.uix.floatlayout import MDFloatLayout
from kivy.properties import StringProperty, NumericProperty, ListProperty, DictProperty, ObjectProperty, BooleanProperty
from kivymd.uix.label import MDLabel
from kivy.graphics import Color, RoundedRectangle
from kivy.metrics import dp

from kivymd.uix.dialog import MDDialog
from kivymd.uix.button import MDFlatButton, MDIconButton
from kivymd.uix.textfield import MDTextField
from kivymd.uix.boxlayout import MDBoxLayout

"""""
Structure of dictionary is as follows:
{
  page_number: {
    zone_id: {
      position_index: {
        "value": function_id,
        "params": {
          "param1": value1,
          "param2": value2,
          ...
        }
      },
      ...
    },
    ...
  },
  ...
}
"""""

page = 1
#dsafg
class DragonDrop(MDFloatLayout):
    #global page
    #page = NumericProperty(1)
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
            {"zone_id": "Alice", "pos_hint": {"x": 0.1, "y": 0.7}, "size_hint": {0.85,0.01}, "bg_color": [0.8, 0.9, 1, 0.3], "split_amount": 7},
            {"zone_id": "Bob",   "pos_hint": {"x": 0.1, "y": 0.45}, "size_hint": {0.85,0.01}, "bg_color": [0.8, 1, 0.8, 0.3], "split_amount": 7},
            {"zone_id": "MiR", "pos_hint": {"x": 0.1, "y": 0.2}, "size_hint": {0.85,0.01}, "bg_color": [1, 0.8, 0.8, 0.3], "split_amount": 7},
        ]

        information = {
            "1":{"param1": "", "param2": ""},
            "2":{"param1": "", "param2": "", "param3": ""},
            "3":{"param1": "", "param2": ""},
            "4":{"param1": "", "param2": ""},
            "5":{"param1": "", "param2": ""},
            "6":{"param1": "", "param2": ""},
        }

        alice = DragonDropZone(
            zone_id=zones[0]["zone_id"],
            pos_hint=zones[0]["pos_hint"],
            size_hint=zones[0]["size_hint"],
            bg_color=zones[0]["bg_color"],
            split_amount=zones[0]["split_amount"],
            #page=self.page,
        )

        bob = DragonDropZone(
            zone_id=zones[1]["zone_id"],
            pos_hint=zones[1]["pos_hint"],
            size_hint=zones[1]["size_hint"],
            bg_color=zones[1]["bg_color"],
            split_amount=zones[1]["split_amount"],
            #page=self.page,
        )

        mir = DragonDropZone(
            zone_id=zones[2]["zone_id"],
            pos_hint=zones[2]["pos_hint"],
            size_hint=zones[2]["size_hint"],
            bg_color=zones[2]["bg_color"],
            split_amount=zones[2]["split_amount"],
            #page=self.page,
        )

        self.add_widget(alice)
        self.add_widget(bob)
        self.add_widget(mir)

        for spec in buttons:
            w = DragonDropButton(
                id_name=spec["id_name"],
                text=spec["text"],
                size_hint=(0.1, 0.1),
                pos_hint={"center_x": spec["center_x"], "top": 0.97},
                font_size=20,
                color=[0.96, 0.96, 0.98, 1],
                bg_color=spec["bg_color"] or [0.5,0.5,0.5,1],
                drop_zones=[alice, bob, mir],
                information=information[spec["id_name"]],
            )
            # set per-button background color if needed:
            # you can add a setter or pass bg_color kwarg; for demo reuse same bg
            self.add_widget(w)

        leftScroll = MDIconButton(
            icon="arrow-left",
            pos_hint={"center_x": 0.05, "top": 0.1},
            on_release=lambda x: self.scroll_left(zones=[alice, bob, mir], buttons=buttons, instance=x)
            )

        rightScroll = MDIconButton(
            icon="arrow-right",
            pos_hint={"center_x": 0.95, "top": 0.1},
            on_release=lambda x: self.scroll_right(zones=[alice, bob, mir], buttons=buttons, instance=x)
        )

        restartButton = MDIconButton(
            icon="restart",
            pos_hint={"center_x": 0.5, "top": 0.1},
            on_release=lambda x: self.reset_all(zones=[alice, bob, mir], visual_only=False)
        )

        self.add_widget(leftScroll)
        self.add_widget(rightScroll)
        self.add_widget(restartButton)

        pageCounter = MDLabel(
            text=f" {page}",
            halign="center",
            theme_text_color="Custom",
            text_color=[0.96, 0.96, 0.98, 1],
            font_style="Subtitle1",
            size_hint=(0.2, 0.1),
            pos_hint={"center_x": 0.02, "top": 0.97},
        )

        self.add_widget(pageCounter)

    def update_page_counter(self, instance):
        # update page counter label
        global page
        for child in self.children:
            if isinstance(child, MDLabel) and child.text.startswith(" "):
                child.text = f" {page}"
                break

    def scroll_left(self, zones, buttons, instance):
        global page
        #print("Scroll left Pressed")
        page -= 1
        #print(f"Switched to page {page}")
        self.reset_all(zones=zones, visual_only=True)
        self._make_visuals_from_dict(zones=zones, buttons=buttons)

    def scroll_right(self, zones, buttons, instance):
        global page
        #print("Scroll right Pressed")
        page += 1
        #print(f"Switched to page {page}")
        self.reset_all(zones=zones, visual_only=True)
        self._make_visuals_from_dict(zones=zones, buttons=buttons)

    def reset_all(self, zones=[], visual_only=False):
        global page
        # remove all VisualCues from parent
        for child in self.children[:]:
            if isinstance(child, VisualCue):
                self.remove_widget(child)

        # clear all zone order dicts
        if not visual_only:
            for zone in zones:
                zone.order.clear()
                page = 1
                #print(f"Cleared zone {zone.zone_id} list.")

        self.update_page_counter(self)

    def _make_visuals_from_dict(self, zones=[], buttons=[]):
        # recreate VisualCues from zone order dicts
        global page
        for zone in zones:
            for page_num in zone.order:
                if page_num != page:
                    continue
            if page not in zone.order:
                continue
            for pos_str, entry in zone.order[page].get(zone.zone_id, {}).items():
                idx = int(pos_str)
                val = entry.get("value")
                params = entry.get("params", {})
                #print(f"Recreating visual for zone {zone.zone_id} at idx {idx} with value {val} and params {params}")
                # create a DragonDropButton to use its method for adding visual
                temp_button = DragonDropButton(
                    id_name=str(val),
                    text=f"Func {val}",
                    color=[0.96, 0.96, 0.98, 1],
                    bg_color=buttons[int(val)-1]["bg_color"] if int(val)-1 < len(buttons) else [0.5,0.5,0.5,1],
                    drop_zones=[zone],
                    information=params,
                )
                temp_button.opacity = 0
                temp_button.disabled = True
                self.add_widget(temp_button)
                try:
                    temp_button.add_visual_in_zone(zone, idx)
                finally:
                    # remove the helper button; visual remains because add_visual_in_zone added it to layout
                    if temp_button.parent is self:
                        self.remove_widget(temp_button)

class DragonDropButton(MDFloatLayout):
    id_name = StringProperty("")
    text = StringProperty("")
    font_size = NumericProperty(18)
    color = ListProperty([0.96, 0.96, 0.98, 1])
    bg_color = ListProperty([0.23, 0.63, 0.92, 1])  # default; override as needed
    drop_zones = ListProperty([])
    information = DictProperty({})

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
                        #print(f"Placing at index: {idx} in zone {zone.zone_id}")
                        #self.add_visual_in_zone(zone, idx)
                        dialog = InfoEncoder(
                            information=self.information,
                            idx=idx,
                            zone=zone,
                            id_name=self.id_name,
                            on_accept=lambda z=zone, i=idx: self.add_visual_in_zone(z, i),
                        )
                        dialog.open()
                        #Only add visual if ok was pressed


                        #self.encode_info(zone, idx)
                        break
                else:
                    print("Not dropped in any zone.")
                    #print(f"list: {zone.order}")
                    #print(f"Alice entry {zone.order.get('Alice')}")
                    #print(f"Bob entry {zone.order.get('Bob')}")
                    #print(f"MiR entry {zone.order.get('MiR')}")

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

    def add_visual_in_zone(self, zone, idx, parent=None):
        if parent is None:
            parent = self.parent

        # Place label in the middle of the zone segment
        #print(f"parent is {parent}")
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
            zone=zone,
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

        self.remove_visual_from_zone(zone, idx)

        # Add the label behind other widgets
        parent.add_widget(label, index=11)

    def remove_visual_from_zone(self, zone, idx, parent=None):
        global page
        # Check parent for VisualCue with matching idx if it is in the zone
        if parent is None:
            parent = self.parent
        for child in parent.children:
            #print(f"Checking child: {child}")
            if isinstance(child, VisualCue) and child.idx == idx and child.zone is zone and zone.order.get(page, {}).get(zone.zone_id, {}).get(str(int(idx))):
                #print(f"There is a visual cue at idx {idx} in zone {zone.zone_id}, removing it.")
                parent.remove_widget(child)
                zone.remove_from_list(idx)

    def reset(self):
        self.pos_hint = self.startpos

class DragonDropZone(DragonDropButton):
    zone_id = StringProperty("")
    split_amount = NumericProperty(None)
    order = dict()
    global page

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Add a label with the zone ID
        label = MDLabel(
            text=self.zone_id,
            halign="left",
            theme_text_color="Custom",
            text_color=[0.96, 0.96, 0.98, 1] ,
            font_style="Subtitle1",
            size_hint=(0.3, 1),
            pos_hint={"x": -0.1, "center_y": 0.5},
        )
        self.add_widget(label)
        #print(f"Created drop zone: {self.zone_id} parent is {self.parent}")

    #Make it not draggable
    def on_touch_down(self, touch):
        #self.page = kwargs.get("page", 1)
        global page
        #print(self.order)
        return False

    def getCoords(self):
        return self.pos_hint
    
    def addToList(self, val, pos, params):
        #print(f"Added to zone {self.zone_id} at pos {pos} value {val} with params {params}")
        global page
        #if self.zone_id not in self.order:
        #    self.order[self.zone_id] = dict()
        #self.order[self.zone_id][str(int(pos))] = {"value": val, "params": deepcopy(params)}
        if page not in self.order:
            self.order[deepcopy(page)] = dict()
        if self.zone_id not in self.order[page]:
            self.order[page][self.zone_id] = dict()
        self.order[page][self.zone_id][str(int(pos))] = {"value": val, "params": deepcopy(params)}

    def remove_from_list(self, pos):
        global page
        if page in self.order and self.zone_id in self.order[page] and str(int(pos)) in self.order[page][self.zone_id]:
            del self.order[page][self.zone_id][str(int(pos))]

class VisualCue(MDLabel):
    #drop_zones = ListProperty([])
    zone = ObjectProperty(None)
    idx = NumericProperty(None)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.zone = kwargs.get("zone")
        self.idx = kwargs.get("idx")

    def on_touch_down(self, touch):
        global page
        touch_x, _ = touch.pos
        cue_center_x, _ = self.center

        edit_params = InfoEncoder(
            information=self.zone.order[page][self.zone.zone_id][str(int(self.idx))]['params'],
            idx=self.idx,
            zone=self.zone,
            id_name=str(self.zone.order[page][self.zone.zone_id][str(int(self.idx))]['value']),
            add_visual=False,
        )

        if self.collide_point(*touch.pos):
            if touch_x >= cue_center_x:
                if self.parent:
                    if self.zone:
                        self.zone.remove_from_list(str(int(self.idx)))
                        #print(f"Zone {zone.zone_id} list now: {zone.order}")
                    else: 
                        print("No zones assigned.")
                    self.parent.remove_widget(self)
            elif touch_x < cue_center_x:
                text_ = f"Function {self.zone.order[page][self.zone.zone_id][str(int(self.idx))]['value']} at position {self.idx}.\n\nParameters:\n"
                for param, value in self.zone.order[page][self.zone.zone_id][str(int(self.idx))]['params'].items():
                    text_ += f" - {param}: {value}\n"
                info_screen = MDDialog(
                    title="Information",
                    text=text_,
                    buttons=[
                        MDFlatButton(
                            text="CLOSE",
                            on_release=lambda x: info_screen.dismiss()
                        ),
                        MDFlatButton(
                            text="EDIT",
                            #open edit dialog and close info dialog
                            on_release=lambda x: (info_screen.dismiss(), edit_params.open())
                        ),
                    ],
                )
                info_screen.open()
            

class InfoEncoder(MDDialog):
    information = DictProperty({})
    idx = NumericProperty(None)
    zone = ObjectProperty(None)
    id_name = StringProperty("")
    add_visual = BooleanProperty(True)

    def __init__(self, **kwargs):
        self._on_accept = kwargs.pop("on_accept", None)
        super().__init__(
            title="Encode Information",
            type="custom",
            content_cls=MDBoxLayout(
                orientation="vertical",
                spacing=dp(10),
                size_hint_y=None,
                height=dp(200),
            ),
            buttons=[
                MDFlatButton(
                    text="CANCEL",
                    on_release=self.dismiss
                    
                ),
                MDFlatButton(
                    text="OK",
                    on_release=self.on_ok
                ),
            ],
            **kwargs
        )

        self.information = kwargs.get("information", [])
        #print(f"InfoEncoder received information: {self.information}")
        self.idx = kwargs.get("idx")
        #print(f"Idx: {self.idx}, Zone: {self.zone.zone_id}, id: {self.id_name}, Info: {self.information}")

        # keep a mapping of param -> widget for the current dialog only
        self._fields = {}

        # ensure the information dict has string placeholders
        for param in list(self.information.keys()):
            # keep the dict value as a string placeholder
            self.information[param] = ""

            # create the input widget and keep a reference
            tf = MDTextField(hint_text=f"{param}")
            self._fields[param] = tf
            self.content_cls.add_widget(tf)

    params = dict()

    def on_ok(self, *args, **kwargs):
        # copy the entered text back into the information dict for the current func
        #func_id = str(int(self.idx))
        # call the stored accept callback (adds the visual) only when OK pressed
        if self.add_visual:
            if callable(getattr(self, "_on_accept", None)):
                try:
                    self._on_accept()
                except Exception as e:
                    print("on_accept callback error:", e)
        n=1
        for param, widget in self._fields.items():
            #print(f"param: {n}, value: {widget.text.strip()}")
            self.params["param"+str(n)] = widget.text.strip()
            n+=1
        
        self.zone.addToList(int(self.id_name), self.idx, self.params)
        self._fields.clear()
        self.params.clear()
        #print(f"List now: {self.zone.order}")

        self.dismiss()

