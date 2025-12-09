from kivymd.uix.boxlayout import MDBoxLayout
from kivymd.uix.label import MDLabel
from kivymd.uix.list import OneLineListItem
from kivy.metrics import dp
from kivy.clock import Clock
from datetime import datetime
import os
import json
from kivymd.uix.button import MDFlatButton

from kivymd.uix.dialog import MDDialog
from kivymd.uix.button import MDRaisedButton
from kivymd.uix.menu import MDDropdownMenu

class StartPage(MDBoxLayout):
    """Start page with system control buttons and status"""
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Reference til main app får vi ved on_parent
        self.app = None
        self.error_messages = []  # Store error messages chronologically
        print("StartPage: __init__ called")
        self._browse_button_added = False  # ensure button only added once
        self._dropdown_menu = None
        self._browse_button = None
        self._last_save_dir = None

        # New: store selected JSON (set when user presses LOAD in file dialog)
        self._selected_json = None
        self._selected_filename = None
    
    def on_parent(self, widget, parent):
        """Called when widget is added to parent - get app reference"""
        if parent:
            from kivymd.app import MDApp
            self.app = MDApp.get_running_app()
            # register with HMI node if available so HMI node can send updates here
            try:
                if hasattr(self.app, 'hmi_node'):
                    self.app.hmi_node.start_page_widget = self
            except Exception:
                pass

            # Bind the START button press to our handler so we can send the selected JSON when START is pressed.
            try:
                if 'start_btn' in self.ids and not getattr(self, "_start_bound", False):
                    self.ids.start_btn.bind(on_release=self._on_start_button_release)
                    self._start_bound = True
            except Exception:
                pass

        print(f"StartPage: Available IDs: {list(self.ids.keys())}")


    def _open_saved_configs_browser(self, *args):
        """Open a dropdown menu anchored to a button that lists files in saved_configs/"""
        save_dir = os.path.join(os.getcwd(), "saved_configs")
        if not os.path.isdir(save_dir):
            dlg = MDDialog(title="No saved_configs directory",
                           text=f"No directory found at: {save_dir}",
                           size_hint=(0.8, 0.3),
                           buttons=[])
            dlg.open()
            return

        files = sorted([f for f in os.listdir(save_dir) if os.path.isfile(os.path.join(save_dir, f))])
        if not files:
            dlg = MDDialog(title="No files",
                           text=f"No files found in: {save_dir}",
                           size_hint=(0.8, 0.3),
                           buttons=[])
            dlg.open()
            return

        # prefer the KV browse button as caller if present
        caller_widget = self.ids.get('browse_btn', None)

        # fallback: create an anchor button once (kept for compatibility)
        if caller_widget is None and not self._browse_button_added:
            self._browse_button = MDRaisedButton(text="Select saved config",
                                                 size_hint=(None, None))
            # try to add button to a sensible container if present, otherwise add to this layout
            if "browse_area" in self.ids:
                self.ids.browse_area.add_widget(self._browse_button)
            else:
                self.add_widget(self._browse_button)
            self._browse_button.bind(on_release=self._open_saved_configs_browser)
            self._browse_button_added = True
            caller_widget = self._browse_button

        # store the directory for later selection handling
        self._last_save_dir = save_dir

        # Build menu items with per-item on_release callbacks (no 'callback' arg)
        menu_items = []
        for fname in files:
            menu_items.append({
                "text": fname,
                "viewclass": "OneLineListItem",
                "height": dp(48),
                "on_release": lambda x=fname: self._on_dropdown_select(x),
            })

        # Dismiss previous menu if exists
        if self._dropdown_menu:
            try:
                self._dropdown_menu.dismiss()
            except Exception:
                pass

        self._dropdown_menu = MDDropdownMenu(
            caller=caller_widget,
            items=menu_items,
            width_mult=6,
            max_height=dp(240),
        )

        # Open the dropdown
        self._dropdown_menu.open()

    def _on_dropdown_select(self, filename):
        """Callback for dropdown menu; filename is the selected filename string"""
        if not filename:
            return

        # dismiss menu
        try:
            if self._dropdown_menu:
                self._dropdown_menu.dismiss()
        except Exception:
            pass

        # show a confirm dialog with LOAD / CANCEL
        file_path = os.path.join(self._last_save_dir or os.path.join(os.getcwd(), "saved_configs"), filename)
        body_text = f"Load JSON file?\n\n{filename}\n\nPath:\n{file_path}"
        def _do_load(*args):
            try:
                with open(file_path, 'r', encoding='utf-8') as fh:
                    contents = fh.read()
                # optionally validate JSON (non-fatal)
                try:
                    json.loads(contents)
                except Exception:
                    # still allow selecting but warn in console
                    print("StartPage: Warning - file is not valid JSON, selecting as-is")

                # Instead of immediately sending, store the JSON as "selected"
                self._selected_json = contents
                self._selected_filename = filename

                # update browse button label to indicate selection if present
                try:
                    if 'browse_btn' in self.ids:
                        short = filename if len(filename) < 24 else ("..." + filename[-21:])
                        self.ids.browse_btn.text = f"SELECTED: {short}"
                except Exception:
                    pass

                # Inform user the file is selected and will be sent when START is pressed
                info = MDDialog(
                    title="File selected",
                    text=f"Selected {filename}\nPress START to send this JSON to the robot.",
                    size_hint=(0.9, 0.35),
                    buttons=[MDFlatButton(text="OK", on_release=lambda *a: info.dismiss())]
                )
                info.open()

            except Exception as e:
                err = MDDialog(title="Failed to load file",
                               text=str(e),
                               size_hint=(0.9, 0.4),
                               buttons=[MDFlatButton(text="OK", on_release=lambda *a: err.dismiss())])
                err.open()
            finally:
                try:
                    print("Skibidi wap pap pap")
                    # confirm_dialog.dismiss()
                except Exception:
                    pass
        
        _do_load()
        # confirm_dialog = MDDialog(
        #     title="Load JSON",
        #     text=body_text,
        #     size_hint=(0.9, 0.5),
        #     buttons=[
        #         MDFlatButton(text="CANCEL", on_release=lambda *a: confirm_dialog.dismiss()),
        #         MDFlatButton(text="LOAD", on_release=_do_load),
        #     ],
        # )
        # confirm_dialog.open()

    def _on_saved_config_selected(self, filename, save_dir, dialog=None):
        """Handle selection of a saved config file (dialog optional)"""
        try:
            if dialog:
                dialog.dismiss()
        except Exception:
            pass

        full_path = os.path.join(save_dir, filename)
        # legacy/backup behavior: show path in a dialog (kept for debug)
        print(f"StartPage: Selected saved config: {full_path}")

    def add_error_message(self, severity, message, node_name):
        """Add error message to chronological list"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        error_text = f"[{timestamp}] {severity} from {node_name}: {message}"
        
        print(f"StartPage: Adding error message: {error_text}")
        
        # Add to our list
        self.error_messages.append(error_text)
        
        # Keep only last 10 messages
        if len(self.error_messages) > 10:
            self.error_messages = self.error_messages[-10:]
        
        # Update UI
        self.update_error_list()
    
    def update_error_list(self):
        """Update the error list in the UI"""
        print(f"StartPage: Updating error list. Available IDs: {list(self.ids.keys())}")
        
        if hasattr(self.ids, 'error_list') and self.ids.error_list:
            print("StartPage: Found error_list widget")
            # Clear existing items
            self.ids.error_list.clear_widgets()
            
            # Add all messages (newest first)
            for error_msg in reversed(self.error_messages):
                item = OneLineListItem(
                    text=error_msg,
                    theme_text_color="Custom",
                    text_color=(0.96, 0.96, 0.98, 1),  # text_light color
                    font_style="Caption"
                )
                self.ids.error_list.add_widget(item)
                print(f"StartPage: Added error item: {error_msg}")
        else:
            print("StartPage: error_list widget not found!")
    
    def clear_error_messages(self):
        """Clear all error messages from the list"""
        print("StartPage: Clearing all error messages")
        self.error_messages.clear()
        
        # Clear the UI list
        if hasattr(self.ids, 'error_list') and self.ids.error_list:
            self.ids.error_list.clear_widgets()
            print("StartPage: Error list cleared from UI")
    
    def _on_start_button_release(self, instance, *args):
        """Handler bound to the START button. If a JSON file is selected, send it via HMI node LoadProgram service."""
        # Keep behavior minimal: if no selection, notify user; if selection exists, call HMI node.
        if not self._selected_json:
            dlg = MDDialog(
                title="No file selected",
                text="Please select a JSON file using BROWSE and press LOAD before starting.",
                size_hint=(0.9, 0.35),
                buttons=[MDFlatButton(text="OK", on_release=lambda *a: dlg.dismiss())]
            )
            dlg.open()
            return

        if not (self.app and hasattr(self.app, 'hmi_node')):
            dlg = MDDialog(
                title="ROS node missing",
                text="HMI node not available to send request.",
                size_hint=(0.9, 0.35),
                buttons=[MDFlatButton(text="OK", on_release=lambda *a: dlg.dismiss())]
            )
            dlg.open()
            return

        try:
            sent = self.app.hmi_node.call_load_raw_JSON_request(self._selected_json, wait_for_service=True)
            if sent:
                dlg = MDDialog(
                    title="JSON sent",
                    text=f"Sent {self._selected_filename or 'selected file'} to LoadProgram service.",
                    size_hint=(0.9, 0.35),
                    buttons=[MDFlatButton(text="OK", on_release=lambda *a: dlg.dismiss())]
                )
            else:
                dlg = MDDialog(
                    title="Send failed",
                    text="Failed to send JSON - check ROS logs.",
                    size_hint=(0.9, 0.35),
                    buttons=[MDFlatButton(text="OK", on_release=lambda *a: dlg.dismiss())]
                )
            dlg.open()
        except Exception as e:
            dlg = MDDialog(
                title="Send error",
                text=str(e),
                size_hint=(0.9, 0.4),
                buttons=[MDFlatButton(text="OK", on_release=lambda *a: dlg.dismiss())]
            )
            dlg.open() 
    
    def system_button_callback(self, action):
        """
        Called by HMIApp.system_button_callback for top-level buttons.
        Handle 'start' by delegating to the page's start handler.
        """
        if action == "start":
            # call the same handler used when the local start button is pressed
            try:
                # pass None as instance because _on_start_button_release expects (instance, *args)
                self._on_start_button_release(None)
            except Exception as e:
                print(f"StartPage: error handling 'start' action: {e}")
        elif action == "clear_errors":
            self.clear_error_messages()
        else:
            print(f"StartPage: unhandled system action: {action}")