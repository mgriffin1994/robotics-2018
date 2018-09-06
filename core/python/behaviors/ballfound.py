"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import pose
import memory
import commands
import cfgstiff
import mem_objects
from state_machine import Node, C, S, T, LoopingStateMachine
import UTdebug



class BallFound(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        print(ball.seen)
        if ball.seen:
            memory.speech.say("ball found")
            print(ball.imageCenterX, ball.imageCenterY)

class Playing(LoopingStateMachine):
    class Off(Node):
        def run(self):
            commands.setStiffness(cfgstiff.Zero)
            if self.getTime() > 2.0:
                # memory.speech.say("turned off stiffness")
                self.finish()


    def setup(self):
        sit = pose.Sit()
        ball_turn = BallFound()
        off = self.Off()

        # self.add_transition(ball_turn, T(4), ball_turn)
        self.add_transition(ball_turn)
        # self.trans(ball_turn, T(4), sit, C, off)
