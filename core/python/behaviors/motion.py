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
            self.finish()

    class TurnBodyLeftInPlace(Node):
        def run(self):
            commands.setWalkVelocity(0, 0, 0.3)

    class TurnBodyRightInPlace(Node):
        def run(self):
            commands.setWalkVelocity(0, 0, -0.9)

    class TurnBodyLeftWalking(Node):
        def run(self):
            commands.setWalkVelocity(0.5, 0, 0.5)

	class Stand(Node):
		def run(self):
			commands.stand()
			self.finish()

    def setup(self):
        sit = pose.Sit()
        head_left = self.TurnHead()
        body_left_in_place = self.TurnBodyLeftInPlace()
        body_right_in_place = self.TurnBodyRightInPlace()
        body_left_walking = self.TurnBodyLeftWalking()
        walk = self.Walk()
        stand = self.Stand()
        off = self.Off()

#         self.trans(stand, C, walk, T(2.0), head_left, C, sit, C, off) # walk, turn head left
#         self.trans(stand, C, body_left_walking, T(4.0), sit, C, off) # walk in a curve
#         self.trans(stand, C, walk, T(2.0), body_left_in_place, T(2.0), sit, C, off) # walk forward and then turn in place

        # all the motions (walk forward, turn left in place, walk and turn left simultaneously, sit, turn head left
        self.trans(stand, C,
                   walk, T(2.0),
                   body_left_in_place, T(2.0),
                   body_left_walking, T(6.0),
                   body_right_in_place, T(5.5),
                   sit, C,
                   head_left, C,
                   off)
