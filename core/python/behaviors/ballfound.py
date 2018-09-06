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
        if ball.seen:
            print(ball.seen, ball.visionBearing)
            print(ball.imageCenterX, ball.imageCenterY)

class Playing(LoopingStateMachine):
    def setup(self):
        ball_turn = BallFound()

        #self.add_transition(ball_turn, T(4), ball_turn)
        self.add_transition(ball_turn)
        # self.trans(ball_turn, T(4), sit, C, off)
