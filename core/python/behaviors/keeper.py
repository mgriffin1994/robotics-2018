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
from memory import joint_commands, localization_mem, robot_state
import UTdebug
import math
import numpy as np

predict_frames = 15
y_thresh = 400
time_delay = 1.0 / 30.
center_region = 125

'''
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
'''


class Blocker(Node):

    def __init__(self):
        super(Blocker, self).__init__()
        self.data = []

    def run(self):

        commands.setStiffness()

        ball = mem_objects.world_objects[core.WO_BALL]

        #if ball.seen:
        #    z = np.array([ball.loc.x, ball.loc.y, ball.absVel.x, ball.absVel.y])
        #    self.data.append(z)
        #    mat = np.array(self.data)
        #    print(mat.shape)
        #    if len(self.data) > 2:
        #        print(np.cov(mat.T))
        #    print()


#        if ball.seen:
#            commands.setHeadPan(ball.bearing, 0.1)

        #'''
        robot = mem_objects.world_objects[robot_state.WO_SELF]
        rob_x = robot.loc.x
        rob_y = robot.loc.y
        ball_pos = localization_mem.getBallPosition()
        x_pos, y_pos = ball_pos.x, ball_pos.y
        ball_vel = localization_mem.getBallVel()
        print()
        print("=== python prints")
        print("ball vel: ", ball.absVel.x, ", ", ball.absVel.y)
        print("ball pos: ", ball.loc.x, ", ", ball.loc.y)
        x_vel, y_vel = ball_vel.x, ball_vel.y
        #friction = localization_mem.getFriction()
#        friction = 0.5 
#        friction = 1.0

        #predicted_x = [x_pos + x_vel * time_delay * ((1 - (friction)**n) / (1 - friction)) for n in range(predict_frames)]
        predicted_x = [x_pos + x_vel * time_delay * n for n in range(predict_frames)]
        #predicted_y = [y_pos + y_vel * time_delay * ((1 - (friction)**n) / (1 - friction)) for n in range(predict_frames)]
        predicted_y = [y_pos + y_vel * time_delay * n for n in range(predict_frames)]

        print("predicted x: ", predicted_x[0], ", ",  predicted_x[-1])
        print("predicted y: ", predicted_y[0], ", ",  predicted_y[-1])
        print("robot x, y: ", rob_x, ", ", rob_y)
        print("velocity x, y: ", x_vel, ", ", y_vel)

        if ball.seen and any(x <= rob_x for x in predicted_x) and any(abs(y - rob_y) < y_thresh for y in predicted_y):
            possible_goal_frame = next(i for i, x in enumerate(predicted_x) if x <= rob_x)
            y_pred = predicted_y[possible_goal_frame]

            if abs(y_pred - rob_y) <= center_region:
                choice = "center"
            elif y_pred > 0:
                choice = "left"
            elif y_pred < 0:
                choice = "right"

            print(choice)
            print()
            self.postSignal(choice)
        #'''


class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        blocks = {"left": pose.BlockLeft(),
                  "right": pose.BlockRight(),
                  "center": pose.BlockCenter()
                  }
        for name in blocks:
            b = blocks[name]
            self.add_transition(blocker, S(name), b, T(3.5), blocker)
# TODO make T back to T(5) when done testing
