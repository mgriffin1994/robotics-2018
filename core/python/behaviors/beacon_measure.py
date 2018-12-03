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
from memory import joint_commands, localization_mem, game_state, robot_state
import UTdebug
import math
import numpy as np
from task import Task

class Blocker(Node):

    def __init__(self):
        super(Blocker, self).__init__()
        self.data = []

    def run(self):
        commands.setStiffness()
        commands.setHeadTilt(-15)
        beacon1 = mem_objects.world_objects[core.WO_BEACON_BLUE_YELLOW]
        if beacon1.seen:
            if len(self.data) < 100:
                self.data.append([beacon1.beacon_height, beacon1.visionBearing])
                mat = np.array(self.data)
                print(mat.shape)
                if len(self.data) > 2:
                    print(np.mean(self.data, axis=0))
                    print(np.sqrt(np.var(self.data, axis=0)))
                print()

        state = game_state
        if state.isPenaltyKick:
            self.data = []
            state.isPenaltyKick = False

class Ready(Task):
    def run(self):
        commands.stand()
        commands.setHeadTilt(-15)
        
class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        self.add_transition(blocker)
