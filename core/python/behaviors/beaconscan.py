"""Sample behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import math
import memory
import pose
import commands
import cfgstiff
from task import Task
from state_machine import Node, C, T, StateMachine



class Playing(StateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                self.finish()

    class TurnBodyLeftInPlace(Node):
        def run(self):
            commands.setWalkVelocity(0, 0, 0.3)


    def setup(self):
        sit = self.Stand()
        body_left_in_place = self.TurnBodyLeftInPlace()

        self.trans(stand, C,
                   body_left_in_place)
