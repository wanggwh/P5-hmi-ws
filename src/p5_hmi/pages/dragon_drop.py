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
import numpy as np

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

class DragonDrop(MDFloatLayout):
    #global page
    #page = NumericProperty(1)
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # pool for reusing VisualCue widgets to avoid frequent create/destroy
        self._visual_pool = []
        self._visual_pool_max = 200
        # map of active visuals for fast lookup: (zone_id, idx) -> VisualCue
        self._visual_map = {}
        # build buttons programmatically
        buttons = [
            {"id_name": "1", "text": "Func A", "center_x": 0.10, "bg_color": [0.23, 0.63, 0.92, 1]},
            {"id_name": "2", "text": "Func B", "center_x": 0.26, "bg_color": [0.11, 0.74, 0.61, 1]},
            {"id_name": "3", "text": "Func C", "center_x": 0.42, "bg_color": [0.54, 0.46, 0.98, 1]},
            {"id_name": "4", "text": "Func D", "center_x": 0.58, "bg_color": [0.98, 0.78, 0.29, 1]},
            {"id_name": "5", "text": "Func E", "center_x": 0.74, "bg_color": [0.94, 0.36, 0.36, 1]},
            {"id_name": "6", "text": "Sync",   "center_x": 0.90, "bg_color": [0.62, 0.65, 0.78, 1]},
        ]

        split_amount = 7

        zones = [
            {"zone_id": "Alice", "pos_hint": {"x": 0.1, "y": 0.7}, "size_hint": {0.85,0.01}, "bg_color": [0.8, 0.9, 1, 0.3], "split_amount": split_amount},
            {"zone_id": "Bob",   "pos_hint": {"x": 0.1, "y": 0.45}, "size_hint": {0.85,0.01}, "bg_color": [0.8, 1, 0.8, 0.3], "split_amount": split_amount},
            {"zone_id": "MiR", "pos_hint": {"x": 0.1, "y": 0.2}, "size_hint": {0.85,0.01}, "bg_color": [1, 0.8, 0.8, 0.3], "split_amount": split_amount},
        ]

        # Create arrays with split_amounts of other arrays
        alice_array = np.empty((split_amount,1), dtype=object)
        which_page_alice = dict()
        bob_array = np.empty((split_amount,1), dtype=object)
        which_page_bob = dict()
        mir_array = np.empty((split_amount,1), dtype=object)
        which_page_mir = dict()

        # storage = np.array([
        #     alice_array,
        #     bob_array,
        #     mir_array,
        # ])

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
            array = alice_array,
            which_page = which_page_alice,
            split_amount=zones[0]["split_amount"],
            #page=self.page,
        )

        bob = DragonDropZone(
            zone_id=zones[1]["zone_id"],
            pos_hint=zones[1]["pos_hint"],
            size_hint=zones[1]["size_hint"],
            bg_color=zones[1]["bg_color"],
            array = bob_array,
            which_page = which_page_bob,
            split_amount=zones[1]["split_amount"],
            #page=self.page,
        )

        mir = DragonDropZone(
            zone_id=zones[2]["zone_id"],
            pos_hint=zones[2]["pos_hint"],
            size_hint=zones[2]["size_hint"],
            bg_color=zones[2]["bg_color"],
            array = mir_array,
            which_page = which_page_mir,
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
        # rebuild visuals in-place to avoid churn
        self.rebuild_visuals(zones=zones, buttons=buttons, page_mappings=[zone.which_page for zone in zones])

    def scroll_right(self, zones, buttons, instance):
        global page
        #print("Scroll right Pressed")
        page += 1
        #print(f"Switched to page {page}")
        # rebuild visuals in-place to avoid churn
        self.rebuild_visuals(zones=zones, buttons=buttons, page_mappings=[zone.which_page for zone in zones])

    def reset_all(self, zones=[], visual_only=False):
        global page
        # remove all VisualCues from parent
        for child in self.children[:]:
            if isinstance(child, VisualCue):
                # prefer releasing into pool if available
                if hasattr(self, 'release_visual'):
                    self.release_visual(child)
                else:
                    self.remove_widget(child)

        # clear all zone order dicts
        if not visual_only:
            for zone in zones:
                zone.array = np.empty((zone.split_amount,1), dtype=object)
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
                # Directly acquire a VisualCue from the pool and add it to layout
                bg = buttons[int(val)-1]["bg_color"] if int(val)-1 < len(buttons) else [0.5,0.5,0.5,1]
                size_hint = (zone.size[0] / zone.split_amount / self.width, 0.2) if self.width else (0.1, 0.2)
                center_x_hint = (zone.pos[0] + (idx - 0.5) * (zone.size[0] / zone.split_amount)) / self.width if self.width else 0
                center_y_hint = (zone.pos[1] + zone.size[1] / 2) / self.height if self.height else 0
                pos_hint = {"center_x": center_x_hint, "center_y": center_y_hint}
                self.acquire_visual(zone=zone, idx=idx, color=[0.96,0.96,0.98,1], bg_color=bg, size_hint=size_hint, pos_hint=pos_hint)

    def _make_visuals_from_array(self, zones=[], buttons=[], page_mappings=[]):
        # Recreate VisualCues from zone arrays and page mappings
        global page
        for i in range(len(zones)):
            zone = zones[i]
            array = zone.array
            which_page = page_mappings[i]
            for placement in range(array.shape[0]):
                for index in range(array.shape[1]):
                    entry = array[placement][index]
                    if entry is not None:
                        # Check if this entry belongs to the current page
                        if which_page.get(page) and (placement, index) in which_page[page]:
                            val = entry.get("value")
                            params = entry.get("params", {})
                            # Directly acquire a VisualCue from the pool and add it to layout
                            bg = buttons[int(val)-1]["bg_color"] if int(val)-1 < len(buttons) else [0.5,0.5,0.5,1]
                            idx_pos = placement + 1
                            size_hint = (zone.size[0] / zone.split_amount / self.width, 0.2) if self.width else (0.1, 0.2)
                            center_x_hint = (zone.pos[0] + (idx_pos - 0.5) * (zone.size[0] / zone.split_amount)) / self.width if self.width else 0
                            center_y_hint = (zone.pos[1] + zone.size[1] / 2) / self.height if self.height else 0
                            pos_hint = {"center_x": center_x_hint, "center_y": center_y_hint}
                            self.acquire_visual(zone=zone, idx=idx_pos, color=[0.96,0.96,0.98,1], bg_color=bg, size_hint=size_hint, pos_hint=pos_hint)

    def _collect_needed_visuals_from_array(self, zones, buttons, page_mappings):
        """Return a list of needed visuals as tuples (zone, idx, bg_color, pos_hint, size_hint)."""
        needed = []
        global page
        for i in range(len(zones)):
            zone = zones[i]
            array = zone.array
            which_page = page_mappings[i]
            for placement in range(array.shape[0]):
                for index in range(array.shape[1]):
                    entry = array[placement][index]
                    if entry is not None:
                        if which_page.get(page) and (placement, index) in which_page[page]:
                            val = entry.get("value")
                            bg = buttons[int(val)-1]["bg_color"] if int(val)-1 < len(buttons) else [0.5,0.5,0.5,1]
                            idx_pos = placement + 1
                            size_hint = (zone.size[0] / zone.split_amount / self.width, 0.2) if self.width else (0.1, 0.2)
                            center_x_hint = (zone.pos[0] + (idx_pos - 0.5) * (zone.size[0] / zone.split_amount)) / self.width if self.width else 0
                            center_y_hint = (zone.pos[1] + zone.size[1] / 2) / self.height if self.height else 0
                            pos_hint = {"center_x": center_x_hint, "center_y": center_y_hint}
                            needed.append((zone, idx_pos, bg, pos_hint, size_hint))
        return needed

    def rebuild_visuals(self, zones, buttons, page_mappings):
        """Reuse existing VisualCue widgets when possible; acquire new ones for missing slots and release extras."""
        # compute needed visuals
        needed = self._collect_needed_visuals_from_array(zones, buttons, page_mappings)
        needed_keys = {(z.zone_id, idx) for (z, idx, *_ ) in needed}

        # map existing visuals by (zone_id, idx)
        existing = {}
        for child in list(self.children):
            if isinstance(child, VisualCue) and getattr(child, 'zone', None) and getattr(child, 'idx', None):
                try:
                    key = (child.zone.zone_id, int(child.idx))
                except Exception:
                    continue
                existing[key] = child

        # Reuse or create visuals for needed slots
        for zone, idx, bg, pos_hint, size_hint in needed:
            key = (zone.zone_id, idx)
            if key in existing:
                v = existing.pop(key)
                # update position and appearance in-place
                try:
                    v.pos_hint = pos_hint
                    v.size_hint = size_hint
                    # update bg color if present
                    if hasattr(v, '_bg_color'):
                        try:
                            v._bg_color.rgba = bg
                        except Exception:
                            pass
                except Exception:
                    pass
            else:
                # acquire a new visual and configure
                self.acquire_visual(zone=zone, idx=idx, color=[0.96,0.96,0.98,1], bg_color=bg, size_hint=size_hint, pos_hint=pos_hint)

        # any visuals left in existing are no longer needed -> release them
        for leftover in existing.values():
            try:
                self.release_visual(leftover)
            except Exception:
                try:
                    if leftover.parent:
                        leftover.parent.remove_widget(leftover)
                except Exception:
                    pass

    # Pool management for VisualCue widgets
    def acquire_visual(self, zone, idx, color, bg_color, size_hint, pos_hint, index=11):
        """Get a VisualCue either from pool or create a new one, add to this layout and return it."""
        if self._visual_pool:
            v = self._visual_pool.pop()
        else:
            v = VisualCue(halign="center", theme_text_color="Custom", font_style="Body1")

        # initialize properties on the visual
        try:
            v.initialize(zone=zone, idx=idx, size_hint=size_hint, pos_hint=pos_hint, color=color, bg_color=bg_color)
        except Exception:
            # fallback: set attributes directly
            v.zone = zone
            v.idx = idx
            v.size_hint = size_hint
            v.pos_hint = pos_hint
            try:
                v.text_color = color
            except Exception:
                pass

        # ensure it's visible and enabled
        v.opacity = 1
        v.disabled = False

        # add back to layout
        self.add_widget(v, index=index)
        # register in visual map for quick lookup
        try:
            self._visual_map[(zone.zone_id, int(idx))] = v
        except Exception:
            pass
        return v

    def release_visual(self, visual):
        """Remove a VisualCue from its parent and store it in the pool for reuse."""
        # unregister from visual map first if possible
        try:
            key = (visual.zone.zone_id, int(visual.idx)) if (visual.zone and visual.idx is not None) else None
        except Exception:
            key = None
        if key and key in getattr(self, '_visual_map', {}):
            try:
                del self._visual_map[key]
            except Exception:
                pass

        try:
            parent = visual.parent
            if parent is not None:
                parent.remove_widget(visual)
        except Exception:
            pass

        # reset visual to a neutral state and store in pool if we haven't reached max
        visual.opacity = 0
        visual.disabled = True
        # detach any heavy references
        try:
            visual.zone = None
        except Exception:
            pass
        try:
            visual.idx = None
        except Exception:
            pass

        if len(self._visual_pool) < getattr(self, "_visual_pool_max", 200):
            # keep the canvas instructions (bg rect) so reuse is cheaper
            self._visual_pool.append(visual)
        # otherwise let it be garbage collected

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
                        #coords = zone.getCoords()
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

        # Create or acquire a VisualCue to represent the dropped button
        size_hint = (zone_width_hint, 0.2)
        pos_hint = {"center_x": center_x_hint, "center_y": center_y_hint}

        # remove existing visual at this slot first
        self.remove_visual_from_zone(zone, idx)

        if hasattr(parent, 'acquire_visual'):
            # let the container manage pooling and adding
            parent.acquire_visual(zone=zone, idx=idx, color=self.color, bg_color=self.bg_color, size_hint=size_hint, pos_hint=pos_hint, index=11)
        else:
            # fallback: create as before
            label = VisualCue(
                halign="center",
                theme_text_color="Custom",
                text_color=self.color,
                font_style="Body1",
                size_hint=size_hint,
                pos_hint=pos_hint,
                zone=zone,
                idx=idx,
            )
            with label.canvas.before:
                Color(*self.bg_color)
                label._bg_rect = RoundedRectangle(
                    pos=label.pos,
                    size=(parent.width * 0.1, dp(24)),
                    radius=[8]
                )

            def _update_bg_rect(instance, value):
                label._bg_rect.pos = instance.pos
                label._bg_rect.size = (instance.width, instance.height)

            label.bind(pos=_update_bg_rect, size=_update_bg_rect)
            parent.add_widget(label, index=11)

    def remove_visual_from_zone(self, zone, idx, parent=None):
        global page
        # Safely get the which_page entries for the given zone/page
        what_did_i_click = []
        if getattr(zone, "which_page", None):
            tmp = zone.which_page.get(page)
            if tmp:
                what_did_i_click = [item for item in tmp if item[0] == idx - 1]

        # Check parent for VisualCue with matching idx if it is in the zone
        if parent is None:
            parent = self.parent

        # If we don't have a mapping for this idx on this page, nothing to remove
        if not what_did_i_click:
            return

        # If the parent provides a fast map, use it for O(1) lookup
        if parent is not None and hasattr(parent, '_visual_map'):
            key = (zone.zone_id, int(idx))
            v = parent._visual_map.get(key)
            if v is not None:
                # verify backing array cell still exists
                page_list_index = what_did_i_click[0][1]
                try:
                    backing = zone.array[idx - 1][page_list_index]
                except Exception:
                    backing = None

                if backing is not None:
                    try:
                        parent.release_visual(v)
                    except Exception:
                        try:
                            if v.parent:
                                v.parent.remove_widget(v)
                        except Exception:
                            pass
                    print("Removing from array at:", what_did_i_click)
                    zone.array = zone.remove_from_array(idx, page_list_index, zone.array)
                return

        # Fallback: iterate children (rare path)
        for child in list(parent.children):
            #print("Checking child:", child)
            try:
                matches = isinstance(child, VisualCue) and child.idx == idx and child.zone is zone
            except Exception:
                matches = False

            if not matches:
                continue

            # verify backing array cell still exists
            page_list_index = what_did_i_click[0][1]
            try:
                backing = zone.array[idx - 1][page_list_index]
            except Exception:
                backing = None

            if backing is not None:
                # prefer releasing into pool if container supports it
                if hasattr(parent, 'release_visual'):
                    parent.release_visual(child)
                else:
                    parent.remove_widget(child)
                print("Removing from array at:", what_did_i_click)
                zone.array = zone.remove_from_array(idx, page_list_index, zone.array)
                # break after first removal
                break

    def reset(self):
        self.pos_hint = self.startpos

class DragonDropZone(DragonDropButton):
    zone_id = StringProperty("")
    split_amount = NumericProperty(None)
    #order = dict()
    array = ObjectProperty(None, force_dispatch=True)
    which_page = ObjectProperty(None, force_dispatch=True)
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
    
    # def addToList(self, val, pos, params):
    #     #print(f"Added to zone {self.zone_id} at pos {pos} value {val} with params {params}")
    #     global page
    #     #if self.zone_id not in self.order:
    #     #    self.order[self.zone_id] = dict()
    #     #self.order[self.zone_id][str(int(pos))] = {"value": val, "params": deepcopy(params)}
    #     if page not in self.order:
    #         self.order[deepcopy(page)] = dict()
    #     if self.zone_id not in self.order[page]:
    #         self.order[page][self.zone_id] = dict()
    #     self.order[page][self.zone_id][str(int(pos))] = {"value": val, "params": deepcopy(params)}

    def addToArray(self, dict, placement, struc):
        global page
        #print(len(struc[device][placement-1]))
        for i in range(len(struc[placement-1])):
            #print(i)
            if struc[placement-1][i] is None:
                struc[placement-1][i] = dict
                if self.which_page.get(page) is None:
                    self.which_page[page] = list()
                self.which_page[page].append((placement-1, i))
                return struc
        struc = np.concatenate((struc, np.empty((self.split_amount,1), dtype=object)), axis=1)
        struc[placement-1][len(struc[placement-1])-1] = dict
        if self.which_page.get(page) is None:
            self.which_page[page] = list()
        self.which_page[page].append((placement-1, len(struc[placement-1])-1))
        return struc

    # def remove_from_list(self, pos):
    #     global page
    #     if page in self.order and self.zone_id in self.order[page] and str(int(pos)) in self.order[page][self.zone_id]:
    #         del self.order[page][self.zone_id][str(int(pos))]

    def remove_from_array(self, placement, pos, struc):
        global page
        if struc[placement-1][pos] is not None:
            struc[placement-1][pos] = None
            if self.which_page.get(page) is not None:
                if (placement-1, pos) in self.which_page[page]:
                    self.which_page[page].remove((placement-1, pos))
        return struc

class VisualCue(MDLabel):
    #drop_zones = ListProperty([])
    zone = ObjectProperty(None, allownone=True)
    idx = NumericProperty(None, allownone=True)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.zone = kwargs.get("zone")
        self.idx = kwargs.get("idx")

    def initialize(self, zone, idx, size_hint, pos_hint, color, bg_color):
        """Prepare or reset this VisualCue for reuse."""
        self.zone = zone
        self.idx = idx
        self.size_hint = size_hint
        self.pos_hint = pos_hint
        self.text_color = color

        # create or update background rect and color
        if not hasattr(self, '_bg_rect'):
            with self.canvas.before:
                self._bg_color = Color(*bg_color)
                self._bg_rect = RoundedRectangle(pos=self.pos, size=(0, 0), radius=[8])

            def _update_bg_rect(instance, value):
                try:
                    self._bg_rect.pos = instance.pos
                    self._bg_rect.size = (instance.width, instance.height)
                except Exception:
                    pass

            self.bind(pos=_update_bg_rect, size=_update_bg_rect)
        else:
            try:
                self._bg_color.rgba = bg_color
            except Exception:
                pass

    def on_touch_down(self, touch):
        global page
        touch_x, _ = touch.pos
        cue_center_x, _ = self.center

        # edit_params = InfoEncoder(
        #     information=self.zone.order[page][self.zone.zone_id][str(int(self.idx))]['params'],
        #     idx=self.idx,
        #     zone=self.zone,
        #     id_name=str(self.zone.order[page][self.zone.zone_id][str(int(self.idx))]['value']),
        #     add_visual=False,
        # )

        what_did_i_click = self.zone.which_page.get(page)
        what_did_i_click = [item for item in what_did_i_click if item[0] == self.idx -1]

        edit_params = InfoEncoder(
            information=self.zone.array[self.idx -1][what_did_i_click[0][1]]['params'],
            idx=self.idx,
            zone=self.zone,
            id_name=str(self.zone.array[self.idx -1][what_did_i_click[0][1]]['value']),
            add_visual=False,
        )

        if self.collide_point(*touch.pos):
            if touch_x >= cue_center_x:
                if self.parent:
                    if self.zone:
                        # index_to_remove = self.zone.which_page.get(page)
                        # index_to_remove = [item for item in index_to_remove if item[0] == self.idx -1]
                        print(f"index to remove: {what_did_i_click}")
                        self.zone.array = self.zone.remove_from_array(self.idx, what_did_i_click[0][1], self.zone.array)
                        print("zone array after removal:\n", self.zone.array)
                        #print(f"Zone {zone.zone_id} list now: {zone.order}")
                    else: 
                        print("No zones assigned.")
                    # release visual back to pool if available
                    if hasattr(self.parent, 'release_visual'):
                        self.parent.release_visual(self)
                    else:
                        self.parent.remove_widget(self)
            elif touch_x < cue_center_x:
                text_ = f"Function {self.zone.array[self.idx -1][what_did_i_click[0][1]]['value']} at position {self.idx}.\n\nParameters:\n"
                for param, value in self.zone.array[self.idx -1][what_did_i_click[0][1]]['params'].items():
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


            # elif touch_x < cue_center_x:
            #     text_ = f"Function {self.zone.order[page][self.zone.zone_id][str(int(self.idx))]['value']} at position {self.idx}.\n\nParameters:\n"
            #     for param, value in self.zone.order[page][self.zone.zone_id][str(int(self.idx))]['params'].items():
            #         text_ += f" - {param}: {value}\n"
            #     info_screen = MDDialog(
            #         title="Information",
            #         text=text_,
            #         buttons=[
            #             MDFlatButton(
            #                 text="CLOSE",
            #                 on_release=lambda x: info_screen.dismiss()
            #             ),
            #             MDFlatButton(
            #                 text="EDIT",
            #                 #open edit dialog and close info dialog
            #                 on_release=lambda x: (info_screen.dismiss(), edit_params.open())
            #             ),
            #         ],
            #     )
            #     info_screen.open()

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
        
        dict = {"value": deepcopy(int(self.id_name)), "params": deepcopy(self.params)}
        self.zone.array = self.zone.addToArray(dict, self.idx, self.zone.array)
        self._fields.clear()
        self.params.clear()
        #print(f"List now: {self.zone.order}")

        print("zone array: \n", self.zone.array, "\nfor zone:", self.zone.zone_id, "\npage mapping:\n", self.zone.which_page)

        self.dismiss()

