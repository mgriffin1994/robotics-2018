"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
from state_machine import Node, S, T, LoopingStateMachine
import UTdebug



class BallFound(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        print(ball.seen)
        if ball.seen:
            print(ball.imageCenterX, ball.imageCenterY)
        if self.getTime() > 1.0:
            self.finish()

class Playing(LoopingStateMachine):
    def setup(self):
        ball_turn = BallFound()
        self.add_transition(ball_turn, C)
