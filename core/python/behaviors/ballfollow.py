"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import pose
import cfgstiff
import commands
import mem_objects
from state_machine import Node, S, T, C, LoopingStateMachine, StateMachine
import UTdebug


time_delay = 0.1

class TurnTowardBall(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        commands.setStiffness()
        if ball.seen:
            # commands.setHeadPanTilt(ball.visionBearing, ball.visionElevation, 0.5)
            commands.setHeadPan(ball.visionBearing, time_delay)
            print(ball.visionBearing, ball.visionElevation)
            print(ball.seen, ball.imageCenterX, ball.imageCenterY)

class Playing(LoopingStateMachine):
#     def run(self):
#         commands.setStiffness()
#         ball = mem_objects.world_objects[core.WO_BALL]
#         if ball.seen:
#             commands.setHeadPan(ball.bearing, time_delay)
#             print(ball.bearing, ball.visionElevation)
#             print(ball.seen, ball.imageCenterX, ball.imageCenterY)
# 
    def setup(self):
        ball_turn = TurnTowardBall()
        # sit = pose.Sit()
        # self.add_transition(ball_turn, T(0.2), ball_turn)
        self.add_transition(ball_turn, T(time_delay), ball_turn)
