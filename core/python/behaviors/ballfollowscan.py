"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import pose
import cfgstiff
import commands
import mem_objects
from state_machine import Node, S, T, C, LoopingStateMachine
import UTdebug
import math
from memory import joint_angles


time_delay = 0.1
eps = 0.01

class TurnTowardBall(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        if ball.seen:
            commands.setHeadPan(ball.visionBearing, time_delay)
            print("=======================================")
            print(ball.visionBearing, ball.visionElevation)
            print(ball.seen, ball.imageCenterX, ball.imageCenterY)
            print("=======================================")
        else:
            self.finish()

class Scan(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        commands.setStiffness()
        if ball.seen:
            self.finish()
        else:
            pan = core.joint_values[core.HeadPan]
            if pan >= 0: #if within eps from full right, turn left
                commands.setHeadPan(-math.pi/4, 3 * time_delay)
            elif pan < 0: #if within eps from full left, turn right
                commands.setHeadPan(math.pi/4, 3 * time_delay)


class Playing(LoopingStateMachine):
 
    def setup(self):
        scan = Scan()
        ball_turn = TurnTowardBall()

        self.add_transition(scan, T(time_delay), ball_turn, C, scan)
