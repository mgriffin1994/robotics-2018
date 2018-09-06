"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
from state_machine import Node, S, T, C, LoopingStateMachine
import UTdebug



class TurnTowardBall(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        if(ball.seen):
            commands.setHeadPan(ball.bearing, 0.5)
            if self.getTime() > 0.5:
                print(ball.seen, ball.imageCenterX, ball.imageCenterY)
            #    self.finish()

class Playing(LoopingStateMachine):
    def setup(self):
        ball_turn = TurnTowardBall()
        self.add_transition(ball_turn, T(0.6), ball_turn)
