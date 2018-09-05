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


class Ready(Task):
    def run(self):
        commands.standStraight()
        if self.getTime() > 5.0:
            memory.speech.say("ready to play")
            self.finish()


class Playing(StateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                memory.speech.say("playing stand complete")
                self.finish()

    class Walk(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0)

    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                memory.speech.say("turned off stiffness")
                self.finish()

    class TurnHead(Node):
        def run(self):
            commands.setHeadPan(math.pi/4, target_time=0.3, isChange=None)
            if self.getTime() > 2.0:
                memory.speech.say("panned head")
                self.finish()

	class Stand(Node):
		def run(self):
			commands.stand()
			self.finish()

    def setup(self):
        sit = pose.Sit()
        head_left = self.TurnHead()
        stand = self.Stand()
        off = self.Off()

        self.trans(sit, C, head_left, C, stand, C, off)
#         self.trans(sit, C, head_left, C, off)
