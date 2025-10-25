# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from ..global_queue_manager import GlobalQueueManager
from .base_command import Command


class Dequeue(Command):
    """
    Command class to dequeue and go to a location after reaching the top of the queue.
    """

    def __init__(self, character, command, navigation_manager, queue_manger: GlobalQueueManager, character_name):
        super().__init__(character, command, navigation_manager)
        self.queue_manager = queue_manger
        self.queue = self.queue_manager.get_queue(command[1])
        self.path = command[2:]
        self.character_name = character_name
        self.command_name = "Dequeue"

    def setup(self):
        super().setup()
        occuiper = self.queue.get_spot(0).get_occupier()

        if occuiper == self.character_name:
            self.queue.get_spot(0).set_occupier(None)
            self.navigation_manager.generate_goto_path(self.path)
            self.character.set_variable("Action", "Walk")
        else:
            self.force_quit_command()

    def update(self, dt):
        self.time_elapsed += dt
        if self.walk(dt):
            return self.exit_command()

    def force_quit_command(self):
        occuiper = self.queue.get_spot(0).get_occupier()
        if occuiper == self.character_name:
            self.queue.get_spot(0).set_occupier(None)
        return super().force_quit_command()
