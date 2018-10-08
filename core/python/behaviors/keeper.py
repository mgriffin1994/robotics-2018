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

predict_frames = 10
y_thresh = 400
time_delay = 0.03

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

        #print(ball_state)
        #print(ball_cov)

#        if ball.seen:
#            commands.setHeadPan(ball.bearing, 0.1)

        robot = mem_objects.world_objects[robot_state.WO_SELF]
        rob_x = robot.loc.x
        rob_y = robot.loc.y
        ball_pos = localization_mem.getBallPosition()
        x_pos, y_pos = ball_pos.x, ball_pos.y
        ball_vel = localization_mem.getBallVel()
        x_vel, y_vel = ball_vel.x, ball_vel.y
        friction = localization_mem.getFriction()

        predicted_x = [x_pos + x_vel * time_delay * ((1 - (friction)**n) / (1 - friction)) for n in range(predict_frames)]
        predicted_y = [y_pos + y_vel * time_delay * ((1 - (friction)**n) / (1 - friction)) for n in range(predict_frames)]
        print()
        print(predicted_x)
        print(predicted_y)

        #TODO: Use prediction equation up to some max number of frames to seen if ball will ever pass goal
        #so see if state of ball.x becomes negative or abs(ball.y) within threshold at any of those times
        #
        if any(x <= rob_x for x in predicted_x) and any(abs(y - rob_y) < y_thresh for y in predicted_y):
            possible_goal_frame = next(i for i, x in enumerate(predicted_x) if x <= rob_x)
            y_pred = predicted_y[possible_goal_frame]

            if abs(y_pred - rob_y) <= 100:
                choice = "center"
            elif y_pred > 0:
                choice = "left"
            elif y_pred < 0:
                choice = "right"

            print(choice)
            print()
            self.postSignal(choice)

#        if ball.distance < 500:
#            UTdebug.log(15, "Ball is close, blocking!")
#            #TODO change these choices to be based on ball velocity and position
#            
#            if ball.bearing > 30 * core.DEG_T_RAD:
#                choice = "left"
#            elif ball.bearing < -30 * core.DEG_T_RAD:
#                choice = "right"
#            else:
#                choice = "center"
#            
#            self.postSignal(choice)
#

class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        blocks = {"left": pose.BlockLeft(),
                  "right": pose.BlockRight(),
                  "center": pose.BlockCenter()
                  }
        for name in blocks:
            b = blocks[name]
            self.add_transition(blocker, S(name), b, T(1.0), blocker)
# TODO make T back to T(5) when done testing
