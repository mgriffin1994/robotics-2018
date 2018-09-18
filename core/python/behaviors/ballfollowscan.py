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
from task import Task


time_delay = 2.0
eps = 0.01

class Ready(Task):
        def run(self):
            #ball = mem_objects.world_objects[core.WO_BALL]
            #if ball.seen:
            #    self.finish()
            #else:
            commands.setStiffness()
            commands.setHeadPan(math.pi/3, time_delay)
            if self.getTime() > time_delay:
               self.finish()

class Playing(LoopingStateMachine):

    class Scan(Node):
        def run(self):
            ball = mem_objects.world_objects[core.WO_BALL]
            if ball.seen:
                self.finish()
            else:
                pan = core.joint_values[core.HeadPan]
                if abs(pan - math.pi / 3) < eps: #if within eps from full right, turn left
                    commands.setHeadPan(-math.pi/3, time_delay)
                elif abs(pan + math.pi / 3) < eps: #if within eps from full left, turn right
                    commands.setHeadPan(math.pi/3, time_delay)
                print("=======================================")
                print(pan)
                print("=======================================")

    class TurnTowardBall(Node):
        def run(self):
            ball = mem_objects.world_objects[core.WO_BALL]
            #commands.setStiffness()
            if ball.seen:
                commands.setHeadPan(ball.visionBearing, time_delay)
                print("=======================================")
                print(ball.visionBearing, ball.visionElevation)
                print(ball.seen, ball.imageCenterX, ball.imageCenterY)
                print("=======================================")
            #else:
            #    pan = core.joint_values[core.HeadPan]
            #    if abs(pan - math.pi / 3) < eps: #if within eps from full right, turn left
            #        commands.setHeadPan(-math.pi/3, time_delay)
            #    elif abs(pan + math.pi / 3) < eps: #if within eps from full left, turn right
            #        commands.setHeadPan(math.pi/3, time_delay)
            #    print("=======================================")
            #    print(pan)
            #    print("=======================================")
            #

    def setup(self):
        #ball_turn = TurnTowardBall()
        scan = self.Scan()

        self.add_transition(scan, T(time_delay), scan)

