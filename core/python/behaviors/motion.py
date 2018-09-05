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

    class TurnBody(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0.0, 0.3)

	class Stand(Node):
		def run(self):
			commands.stand()
			self.finish()

    def setup(self):
        sit = pose.Sit()
        head_left = self.TurnHead()
        body_left = self.TurnBody()
        walk = self.Walk()
        stand = self.Stand()
        off = self.Off()

#         self.trans(stand, C, walk, T(2.0), head_left, C, sit, C, off) # walk, turn head left
        self.trans(stand, C, body_left, T(4.0), sit, C, off) # walk in a curve
#         self.trans(stand, C, walk, T(2.0), body_left, T(2.0), sit, C, off) # walk forward and then turn in place
