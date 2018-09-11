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
    def run(self):
        memory.speech.say('Hello, World!')
        self.finish()
