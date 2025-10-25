# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import __main__
import carb
from omni import ui
from omni.kit.widget.settings import SettingsWidgetBuilder, SettingType, create_setting_widget

from ...settings import PeopleSettings


class NavMeshControlWidget:
    def __init__(self, ext_ui_instance):
        self._ext_ui_instance = ext_ui_instance
        self._checkbox = None
        self._checkbox_label = None

    def shutdown(self):
        self._checkbox = None
        self._checkbox_label = None

    def _build_content(self, title_stack, content_stack):
        with title_stack:
            self._checkbox_label = ui.Label("NavMesh Based Navigation")
        with content_stack:
            with ui.HStack():
                ui.Spacer(width=5)
                widget, model = create_setting_widget(
                    PeopleSettings.NAVMESH_ENABLED,
                    SettingType.BOOL,
                    range_from=0,
                    range_to=0,
                    speed=1,
                    hard_range=False,
                )
                model.set_value(True)
