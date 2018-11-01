"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
import pose
import cfgstiff
from state_machine import Node, S, T, LoopingStateMachine
from memory import joint_commands, localization_mem, robot_state, game_state
from task import Task
import UTdebug
import math
import numpy as np

class Blocker(Node):
    def run(self):
        choice = "left"
        commands.setStiffness()
        self.postSignal(choice)

class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()

        blocks = {"left": pose.BlockLeft(),
                   "right": pose.BlockRight(),
                   "center": pose.SitBlock(),
                   }

        for name in blocks:
            b = blocks[name]
            self.add_transition(blocker, S(name), b, T(10.25), blocker)
