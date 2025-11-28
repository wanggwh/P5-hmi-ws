from copy import deepcopy
import json
import os
from kivymd.uix.floatlayout import MDFloatLayout
from kivy.properties import StringProperty, NumericProperty, ListProperty, DictProperty, ObjectProperty, BooleanProperty
from kivymd.uix.label import MDLabel
from kivy.graphics import Color, RoundedRectangle
from kivy.metrics import dp
from kivymd.app import MDApp

from kivymd.uix.dialog import MDDialog
from kivymd.uix.button import MDFlatButton, MDIconButton
from kivymd.uix.textfield import MDTextField
from kivymd.uix.boxlayout import MDBoxLayout
from kivymd.uix.selectioncontrol import MDCheckbox

"""
Til future jensen:
Lav en relative mover force enabled version knap
Lav admittance knappen til at have en on off for hvert led
"""


page = 1

class DragonDrop(MDFloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        zones = [
            {"zone_id": "Alice", "pos_hint": {"x": 0.1, "y": 0.7}, "size_hint": {0.85,0.01}, "bg_color": [0.8, 0.9, 1, 0.3], "split_amount": 7},
            {"zone_id": "Bob",   "pos_hint": {"x": 0.1, "y": 0.45}, "size_hint": {0.85,0.01}, "bg_color": [0.8, 1, 0.8, 0.3], "split_amount": 7},
            {"zone_id": "MiR", "pos_hint": {"x": 0.1, "y": 0.2}, "size_hint": {0.85,0.01}, "bg_color": [1, 0.8, 0.8, 0.3], "split_amount": 7},
        ]

        # information = {
        #     "1":{"config_name": ""},
        #     "2":{"frame": "", "linear": "", "use_tracking_velocity": "", "pose": ""},
        #     "3":{"frame_name": ""},
        #     "4":{"action": ""},
        #     "5":{"action": ""},
        #     "6":{"sync_id": "", "threads": ""},
        #     "7":{"mission": ""},
        # }

        information = {
            "1":{"func_name": "Config move", "func_args": {"config_name": {"type": "TF", "pretty_name": "Config Name", "ugly_name": "config_name", "extra_text": "", "entry": ""}}},
            "2":{"func_name": "Relative move", "func_args": {
                "frame": {"type": "TF", "pretty_name": "Frame", "ugly_name": "frame", "extra_text": "(for apriltag: tag36h11:X)", "entry": ""}, 
                "linear": {"type": "bool", "pretty_name": "Linear", "ugly_name": "linear", "extra_text": "", "entry": ""}, 
                "use_tracking_velocity": {"type": "bool", "pretty_name": "Use Tracking Velocity", "ugly_name": "use_tracking_velocity", "extra_text": "", "entry": ""}, 
                "pose": {"type": "TF", "pretty_name": "Pose", "ugly_name": "pose", "extra_text": "[x, y, z, qx, qy, qz, qw]", "entry": ""}
                }},
            "3":{"func_name": "Relative move, Force Enabled", "func_args": {
                "frame": {"type": "TF", "pretty_name": "Frame", "ugly_name": "frame", "extra_text": "(for apriltag: tag36h11:X)", "entry": ""}, 
                "linear": {"type": "bool", "pretty_name": "Linear", "ugly_name": "linear", "extra_text": "", "entry": ""}, 
                "use_tracking_velocity": {"type": "bool", "pretty_name": "Use Tracking Velocity", "ugly_name": "use_tracking_velocity", "extra_text": "", "entry": ""}, 
                "pose": {"type": "TF", "pretty_name": "Pose", "ugly_name": "pose", "extra_text": "[x, y, z, qx, qy, qz, qw]", "entry": ""}, 
                "force": {"type": "TF", "pretty_name": "Force Vector", "ugly_name": "force", "extra_text": "I think this is a vector", "entry": ""}}},
            "4":{"func_name": "Frame available", "func_args": {"frame_name": {"type": "TF", "pretty_name": "Frame Name", "ugly_name": "frame_name", "extra_text": "", "entry": ""}}},
            "5":{"func_name": "Grip", "func_args": {"action": {"type": "bool", "pretty_name": "Action", "ugly_name": "action", "extra_text": "(close/open)", "entry": ""}}},
            "6":{"func_name": "Admittance", "func_args": {
                "link_1": {"type": "bool", "pretty_name": "Link 1", "ugly_name": "link_1", "extra_text": "(on/off)", "entry": ""},
                "link_2": {"type": "bool", "pretty_name": "Link 2", "ugly_name": "link_2", "extra_text": "(on/off)", "entry": ""},
                "link_3": {"type": "bool", "pretty_name": "Link 3", "ugly_name": "link_3", "extra_text": "(on/off)", "entry": ""},
                "link_4": {"type": "bool", "pretty_name": "Link 4", "ugly_name": "link_4", "extra_text": "(on/off)", "entry": ""},
                "link_5": {"type": "bool", "pretty_name": "Link 5", "ugly_name": "link_5", "extra_text": "(on/off)", "entry": ""},
                "link_6": {"type": "bool", "pretty_name": "Link 6", "ugly_name": "link_6", "extra_text": "(on/off)", "entry": ""}
                }},
            "7":{"func_name": "Sync", "func_args": {"sync_id": {"type": "TF", "pretty_name": "Sync ID", "ugly_name": "sync_id", "extra_text": "", "entry": ""}, "threads": {"type": "TF", "pretty_name": "Threads", "ugly_name": "threads", "extra_text": "", "entry": ""}}},
            "8":{"func_name": "MiR mission", "func_args": {"mission": {"type": "TF", "pretty_name": "Mission", "ugly_name": "mission", "extra_text": "", "entry": ""}}},
        } 

        alice = DragonDropZone(
            zone_id=zones[0]["zone_id"],
            pos_hint=zones[0]["pos_hint"],
            size_hint=zones[0]["size_hint"],
            bg_color=zones[0]["bg_color"],
            split_amount=zones[0]["split_amount"],
            order=dict(),
        )

        bob = DragonDropZone(
            zone_id=zones[1]["zone_id"],
            pos_hint=zones[1]["pos_hint"],
            size_hint=zones[1]["size_hint"],
            bg_color=zones[1]["bg_color"],
            split_amount=zones[1]["split_amount"],
            order=dict(),
        )

        mir = DragonDropZone(
            zone_id=zones[2]["zone_id"],
            pos_hint=zones[2]["pos_hint"],
            size_hint=zones[2]["size_hint"],
            bg_color=zones[2]["bg_color"],
            split_amount=zones[2]["split_amount"],
            order=dict(),
        )

        # build buttons programmatically
        buttons = [
            {"id_name": "1", "text": "C move", "bg_color": [0.23, 0.63, 0.92, 1], "command": "c_move", "zones": [alice, bob]},
            {"id_name": "2", "text": "R move", "bg_color": [0.11, 0.74, 0.61, 1], "command": "r_move", "zones": [alice, bob]},
            {"id_name": "3", "text": "R move F.E.", "bg_color": [0.11, 0.74, 0.61, 1], "command": "r_move_f_e", "zones": [alice, bob]},
            {"id_name": "4", "text": "Fra. ava.", "bg_color": [0.54, 0.46, 0.98, 1], "command": "frame_available", "zones": [alice, bob, mir]},
            {"id_name": "5", "text": "Grip", "bg_color": [0.98, 0.78, 0.29, 1], "command": "grip", "zones": [alice, bob]},
            {"id_name": "6", "text": "Admit", "bg_color": [0.94, 0.36, 0.36, 1], "command": "admittance", "zones": [alice, bob]},
            {"id_name": "7", "text": "Sync", "bg_color": [0.86, 0.52, 0.60, 1], "command": "sync", "zones": [alice, bob, mir]},
            {"id_name": "8", "text": "MiR", "bg_color": [0.62, 0.65, 0.78, 1], "command": "mir_move", "zones": [mir]},
        ]

        self.add_widget(alice)
        self.add_widget(bob)
        self.add_widget(mir)

        for spec in buttons:
            w = DragonDropButton(
                id_name=spec["id_name"],
                text=spec["text"],
                size_hint=(0.1, 0.1),
                pos_hint={"center_x": 0.08 + 0.84/(len(buttons)-1)*(int(spec["id_name"])-1), "top": 0.97},
                font_size=20,
                color=[0.96, 0.96, 0.98, 1],
                bg_color=spec["bg_color"] or [0.5,0.5,0.5,1],
                drop_zones=spec["zones"],
                information=information[spec["id_name"]],
            )
            self.add_widget(w)
            #print(f"\n\nButton {int(spec["id_name"])}\nplacement:\n", 0.10 + 0.8/(len(buttons)-1)*(int(spec["id_name"])-1))

        leftScroll = MDIconButton(
            icon="arrow-left",
            pos_hint={"center_x": 0.85, "center_y": 0.05},
            on_release=lambda x: self.scroll_left(zones=[alice, bob, mir], buttons=buttons, instance=x),
            theme_icon_color="Custom",
            icon_color =[0.86, 0.86, 0.88, 1],
            )

        rightScroll = MDIconButton(
            icon="arrow-right",
            pos_hint={"center_x": 0.90, "center_y": 0.05},
            on_release=lambda x: self.scroll_right(zones=[alice, bob, mir], buttons=buttons, instance=x),
            theme_icon_color="Custom",
            icon_color =[0.86, 0.86, 0.88, 1],
        )

        restartButton = MDIconButton(
            icon="restart",
            pos_hint={"center_x": 0.05, "center_y": 0.05},
            on_release=lambda x: self.reset_all(zones=[alice, bob, mir], visual_only=False),
            theme_icon_color="Custom",
            icon_color =[0.86, 0.86, 0.88, 1],
        )

        showdictButton = MDIconButton(
            icon="printer-outline",
            pos_hint={"center_x": 0.1, "center_y": 0.05},
            on_release=lambda x: print(f"Current zone orders:\n\nAlice: {alice.order}\n\nBob: {bob.order}\n\nMiR: {mir.order}\n"),
            theme_icon_color="Custom",
            icon_color =[0.86, 0.86, 0.88, 1],
        )

        saveAndParse = MDIconButton(
            icon="content-save",
            pos_hint={"center_x": 0.15, "center_y": 0.05},
            on_release=lambda x: self._parse_json(zones=[alice, bob, mir], buttons=buttons),
            theme_icon_color="Custom",
            icon_color =[0.86, 0.86, 0.88, 1],
        )

        self.add_widget(leftScroll)
        self.add_widget(rightScroll)
        self.add_widget(restartButton)
        self.add_widget(showdictButton)
        self.add_widget(saveAndParse)

        pageCounter = MDLabel(
            text=f" {page}",
            halign="center",
            theme_text_color="Custom",
            text_color=[0.86, 0.86, 0.88, 1],
            font_style="Subtitle1",
            size_hint=(0.05, 0.1),
            pos_hint={"center_x": 0.95, "center_y": 0.05},
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

    def _parse_json(self, zones=[], buttons=[]):
        global page

        json_data = {
            "NAME": {
                "description": "",
                "date": "",
                "author": "",
                "threads": [{
                    "name": "alice_thread",
                    "robot_name": "alice",
                    "commands": [

                    ],
                },
                {
                    "name": "bob_thread",
                    "robot_name": "bob",
                    "commands": [

                    ],
                },
                {
                    "name": "mir_thread",
                    "robot_name": "mir",
                    "commands": [

                    ],
                }
                ]
            }
        }

        # Make MDDialog with textfields to enter Name, description, date, author
        naming = {
            "name": "",
            "description": "",
            "date": "",
            "author": "",
        }
        naming_dialog = MDDialog(
            title="Save and Parse",
            type="custom",
            content_cls=MDBoxLayout(
                orientation="vertical",
                spacing=dp(10),
                size_hint_y=None,
                height=len(naming) * dp(70),
            ),
            buttons=[
                MDFlatButton(text="CANCEL", on_release=lambda x: naming_dialog.dismiss()),
                MDFlatButton(text="OK", on_release=lambda x: on_ok(x)),
            ],
        )
        for param in naming:
            tf = MDTextField(hint_text=f"{param}")
            naming_dialog.content_cls.add_widget(tf)
        naming_dialog.open()

        def on_ok(instance):
            for i, param in enumerate(naming):
                naming[param] = naming_dialog.content_cls.children[len(naming) - 1 - i].text.strip()

            #print(f"Saving with parameters: {naming}")

            # 1. Extract the data currently under "NAME"
            json_data[naming["name"]] = json_data.pop("NAME")

            # 2. Fill in user-provided info
            json_data[naming["name"]]["description"] = naming["description"]
            json_data[naming["name"]]["date"] = naming["date"]
            json_data[naming["name"]]["author"] = naming["author"]

            #print(f"json_data after naming: {json_data}")

            # 3. For each zone, extract its order dict and fill in commands
            for zone in zones:
                print(f"zone {zone.zone_id} order: {zone.order}")
                zone_id = zone.zone_id.lower()
                thread_entry = next((t for t in json_data[naming["name"]]["threads"] if t["robot_name"] == zone_id), None)
                if thread_entry is None:
                    continue

                # Sort the entries by position index and page number
                sorted_entries = []
                for page_num in sorted(zone.order.keys()):
                    for pos_str in sorted(zone.order[page_num].get(zone.zone_id, {}).keys(), key=lambda x: int(x)):
                        entry = zone.order[page_num][zone.zone_id][pos_str]
                        sorted_entries.append((page_num, int(pos_str), entry))
                for page_num, pos_idx, entry in sorted_entries:
                    print("\n\nAdding entry:", entry, "\n\n")
                    func_id = entry.get("value")
                    params = entry.get("params", {})
                    args = {entry.get("ugly_name"): entry.get("entry") for entry in params.values()}
                    command = {
                        "command": buttons[int(func_id)-1]["command"],
                        "args": args,
                    }
                    thread_entry["commands"].append(command)

            #print(f"Final JSON data:\n{json_data}")

            # Remove empty threads
            for thread in json_data[naming["name"]]["threads"][:]:
                #print(f"thread {thread['name']} has {len(thread['commands'])} commands.\n\n")
                if len(thread["commands"]) == 0:
                    #print(f"Removing empty thread: {thread['name']}")
                    json_data[naming["name"]]["threads"].remove(thread)

            #print(f"\n\nFinal JSON data:\n\n{json_data}\n\n")
            json_string = json.dumps(json_data, indent=4)
            print(f"JSON String:\n\n{json_string}\n\n")
            
            # def _on_payload_ready(self, json_str):
            #     try:
            #         payload = json.loads(json_str)
            #         if not payload:
            #             print("DragonDrop: empty payload")
            #             return

            #         # Top-level NAME key is the program name; everything under it is the program content
            #         program_name = next(iter(payload.keys()))
            #         print(f"DragonDrop: program name '{program_name}'")
            #         program_content = payload[program_name]
            #         program_json = json.dumps(program_content)
            #         print(f"DragonDrop: program JSON content:\n{program_json}")

            #         app = MDApp.get_running_app()
            #         if app and hasattr(app, "hmi_node") and app.hmi_node:
            #             # Call the fresh, non-reused client to save the program
            #             app.hmi_node.call_save_program_request(program_name, program_json, wait_for_service=True)
            #             print(f"DragonDrop: called save_program for '{program_name}'")
            #         else:
            #             print("DragonDrop: HMI node not available (make sure app.hmi_node is set)")
            #     except Exception as e:
            #         print("DragonDrop: failed to parse/call save_program service:", e)

            #_on_payload_ready(self, json_string)

            save_it = MDDialog(
                title="Save file?",
                text="Contents can be found in console output. Save to file?",
                size_hint=(0.9, 0.9),
                buttons=[
                    MDFlatButton(text="NO", on_release=lambda x: save_it.dismiss()),
                    MDFlatButton(text="YES", on_release=lambda x: save_it_on_ok(x)),
                ],
            )
            save_it.open()

            def save_it_on_ok(instance):
                # Here you would implement actual file saving if desired
                save_dir = "saved_configs"
                os.makedirs(save_dir, exist_ok=True)
                file_path = os.path.join(save_dir, f"{naming['name']}.json")

                with open(file_path, "w") as f:
                    f.write(json_string)
                save_it.dismiss()

            naming_dialog.dismiss()

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
    order = ObjectProperty(None)
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
        if not self.collide_point(*touch.pos):
            return

        touch_x = touch.x
        cue_center_x = self.center_x

        # Right side: delete
        if touch_x >= cue_center_x:
            if self.parent and self.zone:
                self.zone.remove_from_list(str(int(self.idx)))
                self.parent.remove_widget(self)
            return
        # Left side: show info dialog; create editor lazily only if EDIT pressed
        entry = self.zone.order.get(page, {}).get(self.zone.zone_id, {}).get(str(int(self.idx)), {})
        value = entry.get('value', '?')
        params = entry.get('params', {})

        # Try to find the original button to get its func_name (so edit dialog shows it)
        func_name = None
        if self.parent:
            for child in self.parent.children:
                # DragonDropButton instances carry the original spec in `.information`
                if isinstance(child, DragonDropButton) and str(child.id_name) == str(value):
                    info = getattr(child, "information", {})
                    if isinstance(info, dict):
                        func_name = info.get("func_name")
                    break

        # build text with join (faster than repeated concatenation)
        header = f"Function {func_name} at position {self.idx}.\n\nParameters:"
        param_lines = [f" - {v.get('pretty_name', p)}: {v.get('entry', '')}" for p, v in params.items()]
        text_ = "\n".join([header] + param_lines)

        info_screen = MDDialog(
            title="Information",
            text=text_,
            buttons=[
                MDFlatButton(text="CLOSE", on_release=lambda x: info_screen.dismiss()),
                MDFlatButton(
                    text="EDIT",
                    # create and open InfoEncoder only when EDIT pressed
                    on_release=lambda x: (
                        info_screen.dismiss(),
                        InfoEncoder(
                            # wrap params so InfoEncoder gets func_name + func_args shape
                            information={"func_name": func_name or f"Function {value}", "func_args": params},
                            idx=self.idx,
                            zone=self.zone,
                            id_name=str(value),
                            add_visual=False,
                        ).open()
                    )
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
        # keep accept callback if provided
        self._on_accept = kwargs.pop("on_accept", None)
        # store incoming info but avoid creating input widgets immediately
        raw = kwargs.get("information", {}) or {}
        self.information = raw
        self.idx = kwargs.get("idx")
        self.zone = kwargs.get("zone")
        self.id_name = kwargs.get("id_name", "")
        self.add_visual = kwargs.get("add_visual", True)

        # Support two shapes for `information`:
        # - button spec: {"func_name": "...", "func_args": {...}}
        # - params dict (editing existing): {"param": {...}, ...}
        if isinstance(raw, dict) and "func_args" in raw and isinstance(raw["func_args"], dict):
            self._func_args_source = raw["func_args"]
        elif isinstance(raw, dict) and all(isinstance(v, dict) for v in raw.values()):
            # already a params dict
            self._func_args_source = raw
        else:
            self._func_args_source = {}

        num_fields = max(1, len(self._func_args_source))

        # create the dialog shell; content_cls exists but is left empty for now
        super().__init__(
            title=raw.get("func_name") or f"Edit {self.id_name}" or "Parameters",
            type="custom",
            content_cls=MDBoxLayout(
                orientation="vertical",
                spacing=dp(10),
                size_hint_y=None,
                height=num_fields * dp(60),
            ),
            buttons=[
                MDFlatButton(text="CANCEL", on_release=self.dismiss),
                MDFlatButton(text="OK", on_release=self.on_ok),
            ],
            **kwargs
        )

        # fields will be created lazily when the dialog is opened
        self._fields = {}
        self._built = False

    def build_fields(self):
        """Create input widgets once when needed."""
        # ensure building only once
        if self._built:
            return

        # Normalize the function-args source so every entry is a dict
        normalized = {}
        for param, val in list(self._func_args_source.items()):
            if isinstance(val, dict) and "type" in val:
                info = val.copy()
                info.setdefault("pretty_name", param)
                info.setdefault("entry", "" if info.get("type") == "TF" else bool(info.get("entry", False)))
            else:
                # val is a plain value (older saved shape)
                if isinstance(val, bool):
                    info = {"type": "bool", "pretty_name": param, "entry": bool(val)}
                else:
                    info = {"type": "TF", "pretty_name": param, "entry": "" if val is None else str(val)}
            normalized[param] = info

        # keep normalized param dict available for on_ok
        self._params = normalized

        # create widgets from normalized info
        for param, info in self._params.items():
            ptype = info.get("type", "TF")
            pretty = info.get("pretty_name", param)
            extra = info.get("extra_text", "")
            if ptype == "TF":
                info["entry"] = info.get("entry", "")
                tf = MDTextField(hint_text=f"{pretty} {extra}".strip(), text=info.get("entry", ""))
                self._fields[param] = tf
                self.content_cls.add_widget(tf)
            elif ptype == "bool":
                info["entry"] = bool(info.get("entry", False))
                row = MDBoxLayout(orientation="horizontal", spacing=dp(8), size_hint_y=None, height=dp(48))
                lbl = MDLabel(
                    text=f"{pretty} {extra}".strip(),
                    halign="left",
                    valign="center",
                    theme_text_color="Custom",
                    text_color=[0.65, 0.65, 0.65, 1],
                )
                cb = MDCheckbox(active=info["entry"])
                row.add_widget(lbl)
                row.add_widget(cb)
                self._fields[param] = cb
                self.content_cls.add_widget(row)

        self._built = True

    def open(self, *args, **kwargs):
        # build the content lazily before showing
        self.build_fields()
        return super().open(*args, **kwargs)

    params = dict()

    def on_ok(self, *args, **kwargs):
        # create visual (if desired) and copy fields
        if self.add_visual and callable(getattr(self, "_on_accept", None)):
            try:
                self._on_accept()
            except Exception as e:
                print("on_accept callback error:", e)

        # Build a structured params dict (preserve type/pretty_name) before storing.
        stored_params = {}
        params_source = getattr(self, "_params", {}) or {}
        for param, widget in self._fields.items():
            # determine value from widget
            if isinstance(widget, MDCheckbox):
                val = bool(widget.active)
            else:
                val = widget.text.strip()

            # preserve the normalized info dict where available, otherwise create a fallback
            orig = params_source.get(param)
            if isinstance(orig, dict):
                info = orig.copy()
            else:
                info = {"type": "TF", "pretty_name": param, "entry": ""}

            # store the user-entered value in the entry field
            info["entry"] = val
            stored_params[param] = info

        # store structured params so pretty_name and type are preserved for future edits
        self.zone.addToList(int(self.id_name), self.idx, stored_params)
        self._fields.clear()
        self.params.clear()
        self.dismiss()