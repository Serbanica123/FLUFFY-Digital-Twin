# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import asyncio

import carb
import omni.ext
import omni.kit.app
import omni.ui

from .scripts.custom_command.command_manager import CustomCommandManager
from .ui import BehaviorControlUI

_extension_instance = None


def get_instance():
    return _extension_instance


class Main(omni.ext.IExt):
    def on_startup(self, ext_id):
        carb.log_info("[omni.anim.people] startup")
        global _extension_instance
        _extension_instance = self
        # Custom command manager
        self._cmd_manager = CustomCommandManager()
        self._cmd_manager.startup()
        # Build UI
        self._behavior_ui = None

        async def build():
            await omni.kit.app.get_app().next_update_async()
            self._behavior_ui = BehaviorControlUI(self)

        asyncio.ensure_future(build())

    def on_shutdown(self):
        carb.log_info("[omni.anim.people] shutdown")
        global _extension_instance
        _extension_instance = None
        # Shutdown Behavior Control UI
        if self._behavior_ui:
            self._behavior_ui.shutdown()
            self._behavior_ui = None

        self._cmd_manager.shutdown()
        self._cmd_manager = None

    def get_custom_command_manager(self):
        return self._cmd_manager
