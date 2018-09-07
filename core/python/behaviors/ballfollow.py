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
        commands.setStiffness()
        if(ball.seen):
            commands.setHeadPanTilt(ball.bearing, 0.3)
            print(ball.seen, ball.imageCenterX, ball.imageCenterY)

class Playing(LoopingStateMachine):
    def setup(self):
        ball_turn = TurnTowardBall()
        self.add_transition(ball_turn, T(0.3), ball_turn)
