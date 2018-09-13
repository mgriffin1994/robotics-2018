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
            print(ball.visionBearing, ball.visionElevation)
            print(ball.seen, ball.imageCenterX, ball.imageCenterY)
        else:
            self.finish() #if lose track of ball repeat loop

class LookLeft(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        commands.setStiffness()
        commands.setHeadPan(-math.pi/2, time_delay) #default looking left
        if ball.seen: #if can see ball at any time during this break out
            self.finish()
        if self.getTime() > time_delay: #if over time_delay seconds from start of this node end (done moving head)
            self.finish()

class Scan(Node):
    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        if ball.seen:
            self.finish()
        else:
            float pan = core.joint_values[core.HeadPan]
            if (abs(pan - math.pi/2) < eps): #if within eps from full right, turn left
                commands.setHeadPan(-math.pi/2, time_delay)
            elif(abs(pan  + math.pi/2) < eps): #if within eps from full left, turn right
                commands.setHeadPan(math.pi/2, time_delay)


class Playing(LoopingStateMachine):
 
    def setup(self):
        start_scan = LookLeft()
        scan = Scan()
        ball_turn = TurnTowardBall()

        self.add_transition(start_scan, C, scan, C, ball_turn, C)
