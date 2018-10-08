"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
import pose
import cfgstiff
from state_machine import Node, S, T, LoopingStateMachine
from memory import joint_commands, localization_mem
import UTdebug
import math


# TODO: change shoulder rolls/pitches with joint_commands.setJointAngle(...)
#		Use enum from common/RobotInfo.h
class BlockLeft(Node):
    def run(self):
        UTdebug.log(15, "Blocking left")
        #joint_commands.send_arm_angles_ = True
        #joint_commands.setJointCommand(core.LShoulderRoll, math.pi/3)


class BlockRight(Node):
    def run(self):
        UTdebug.log(15, "Blocking right")
        #joint_commands.send_arm_angles_ = True
        #joint_commands.setJointCommand(core.RShoulderRoll, math.pi/3)



class BlockCenter(Node):
    def run(self):
        UTdebug.log(15, "Blocking right")
        #joint_commands.send_arm_angles_ = True
        #joint_commands.setJointCommand(core.RShoulderPitch, math.pi/3)
        #joint_commands.setJointCommand(core.LShoulderPitch, math.pi/3)



class Blocker(Node):
    def run(self):

        commands.setStiffness()

        ball = mem_objects.world_objects[core.WO_BALL]
        #ball_state = localization_mem.state
        #ball_cov = localization_mem.covariance
        #print(localization_mem.getBallPosition(), localization_mem.getBallVel(), localization_mem.getFriction())

        #print(ball_cov)

        #TODO grab the ball state as well
        if ball.seen:
            commands.setHeadPan(ball.bearing, 0.1)
		# TODO: Use velocity vector to determine if goal or not
        if ball.distance < 500:
            UTdebug.log(15, "Ball is close, blocking!")
            #TODO change these choices to be based on ball velocity and position
            
            if ball.bearing > 30 * core.DEG_T_RAD:
                choice = "left"
            elif ball.bearing < -30 * core.DEG_T_RAD:
                choice = "right"
            else:
                choice = "center"
            
            self.postSignal(choice)


class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        blocks = {"left": BlockLeft(),
                  "right": BlockRight(),
                  "center": BlockCenter()
                  }
        for name in blocks:
            b = blocks[name]
            self.add_transition(blocker, S(name), b, T(8), blocker)
# TODO make T back to T(5) when done testing