# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os

import __main__
import carb
from omni import ui
from omni.anim.people.scripts.custom_command.command_manager import (
    CustomCommand,
    CustomCommandManager,
    CustomCommandTemplate,
)
from omni.kit.window.filepicker import FilePickerDialog

from ..collapsable_widget import CollapsableWidget


class CustomCommandPanel(CollapsableWidget):
    def __init__(self, ext_ui_instance, ext_instance):
        super().__init__("Custom Commands")
        self.cmd_manager: CustomCommandManager = ext_instance.get_custom_command_manager()
        self.file_picker = None

    def shutdown(self):
        super().shutdown()

    def _build_content(self):
        with ui.VStack():
            # with ui.HStack():
            # ui.Label("Registered Commands")
            with ui.ScrollingFrame(
                height=300,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self._custom_commands_model = CustomCommandPanel.UICustomCommandModel(self.cmd_manager)
                self._custom_commands_delegate = CustomCommandPanel.UICustomCommandDelegate()
                self._tree_view = ui.TreeView(
                    self._custom_commands_model,
                    delegate=self._custom_commands_delegate,
                    column_widths=[ui.Fraction(0.25), ui.Fraction(0.25), ui.Fraction(0.5)],
                    root_visible=False,
                    header_visible=True,
                )
            ui.Spacer(height=5)
            with ui.HStack(spacing=30):
                self._add_btn = ui.Button("Add", width=160)
                self._remove_btn = ui.Button("Remove", width=160)
                self._add_btn.set_clicked_fn(self._on_add_btn)
                self._remove_btn.set_clicked_fn(self._on_remvoe_btn)

    def _on_add_btn(self):
        # Avoid opening multiple file picker (Not sure if gc will take care of it)
        if self.file_picker is not None:
            self.file_picker.hide()
            self.file_picker = None

        def on_selected(filename, path):
            if filename == None or filename == "":
                return
            full_path = os.path.join(path, filename)
            if self.cmd_manager.add_custom_command(full_path):
                self.cmd_manager.notify_listeners("ADD_COMMAND", self.cmd_manager.get_latest_command())
            self._custom_commands_model.fetch_commands()
            if self.file_picker:
                self.file_picker.hide()
                self.file_picker = None

        def on_canceled(a, b):
            if self.file_picker:
                self.file_picker.hide()
                self.file_picker = None

        def filter_usd(item):
            if not item or item.is_folder:
                return True
            return item.path.endswith(".usd") or item.path.endswith(".usda")

        self.file_picker = FilePickerDialog(
            "Add a custom command animation USD",
            allow_multi_selection=False,
            apply_button_label="Select",
            click_apply_handler=lambda a, b: on_selected(a, b),
            click_cancel_handler=lambda a, b: on_canceled(a, b),
            item_filter_fn=filter_usd,
            file_extension_options=[("*.usd; *.usda", "Universal Scene Description")],
            enable_versioning_pane=True,
        )

    def _on_remvoe_btn(self):
        if len(self._tree_view.selection) == 0:
            carb.log_error("Please select a custom command to remove.")
            return
        anim_path = self._tree_view.selection[0].anim_usd_model.get_value_as_string()
        command_to_remove = self.cmd_manager.get_command_by_anim_path(anim_path)
        self.cmd_manager.notify_listeners("REMOVE_COMMAND", command_to_remove)
        self.cmd_manager.remove_custom_command(anim_path)
        self._custom_commands_model.fetch_commands()

    class UICustomCommand(ui.AbstractItem):
        def __init__(self, item: CustomCommand):
            super().__init__()
            self.name_model = ui.SimpleStringModel(item.name)
            self.template_model = ui.SimpleStringModel(item.template.value)
            self.anim_usd_model = ui.SimpleStringModel(item.anim_path)

        def __repr__(self):
            return f'"{self.name_model.as_string} {self.template_model.as_string} {self.anim_usd_model.as_string}"'

    class UICustomCommandModel(ui.AbstractItemModel):
        def __init__(self, cmd_manager):
            super().__init__()
            self._children = []
            self.cmd_manager = cmd_manager
            self.fetch_commands()

        def fetch_commands(self):
            self._children.clear()
            items_list = self.cmd_manager.get_all_custom_commands()
            self._children = [CustomCommandPanel.UICustomCommand(item) for item in items_list]
            self._item_changed(None)

        def get_item_children(self, item):
            if item is not None:
                return []
            return self._children

        def get_item_value_model_count(self, item):
            return 3  # Column count

        def get_item_value_model(self, item, column_id):
            model = None
            if column_id == 0:
                model = item.name_model
            elif column_id == 1:
                model = item.template_model
            elif column_id == 2:
                model = item.anim_usd_model
            return model

    class UICustomCommandDelegate(ui.AbstractItemDelegate):
        def __init__(self):
            super().__init__()

        def build_branch(self, model, item, column_id, level, expanded):
            pass

        def build_header(self, column_id):
            if column_id == 0:
                ui.Label("Command Name", height=30)
            elif column_id == 1:
                ui.Label("Template", height=30)
            else:
                ui.Label("USD Path", height=30)

        def build_widget(self, model, item, column_id, level, expanded):
            stack = ui.ZStack(height=20)
            with stack:
                value_model = model.get_item_value_model(item, column_id)
                label = ui.Label(value_model.as_string)
