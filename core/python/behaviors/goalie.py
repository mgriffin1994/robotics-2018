"""Simple goalie behavior."""

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

predict_secs = 20
y_thresh = 400
time_delay = 1.0 / 30.
center_region = 125
x_thresh = 50

# TODO:
#   - Move to center of penalty box using localization/vision
#   - Test different blocks
#       - Develop different squat block, the one in pose.py sucks
#   - Use Kalman filter to predict ball location and move, if necessary
#       - Speed up prediction, blocking is too slow
#   - Choose block to use (may need more than the three provided)

# TODO:
#  - could do open loop control (hack) to keep ball in center of view
#    - keep track of the location of the ball
#    - if it moves outside some threshold that is considered as the center of view
#      - move in that direction by a small amount until the ball is in the center again

class Blocker(Node):

    def __init__(self):
        super(Blocker, self).__init__()
        self.data = []

    def run(self):
        commands.setStiffness()

        ball = mem_objects.world_objects[core.WO_BALL]
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

        #if ball.seen:
        #    angle = np.arctan((y_pos - rob_y)/(x_pos - rob_x + 1e-5))
        #    commands.setHeadPan(angle, 0.1)
        #else:
        #    commands.setHeadPan(0, 0.1)

        predicted_x = [x_pos + x_vel * time_delay * n for n in np.arange(0.0, predict_secs, 0.1)]
        predicted_y = [y_pos + y_vel * time_delay * n for n in np.arange(0.0, predict_secs, 0.1)]

        print("predicted x: ", predicted_x[0], ", ",  predicted_x[-1])
        print("predicted y: ", predicted_y[0], ", ",  predicted_y[-1])
        print("robot x, y: ", rob_x, ", ", rob_y)
        print("velocity x, y: ", x_vel, ", ", y_vel)

        if ball.seen and any(x <= rob_x - x_thresh for x in predicted_x):
            possible_goal_frames = [i for i, x in enumerate(predicted_x) if x <= rob_x - x_thresh]
            if any(abs(predicted_y[i]-rob_y) < y_thresh for i in possible_goal_frames):
                y_pred = predicted_y[possible_goal_frames[0]]
                print('num_frames', possible_goal_frames[0])
                if abs(y_pred - rob_y) <= center_region:
                    choice = "center"
                elif y_pred > 0:
                    choice = "left"
                elif y_pred < 0:
                    choice = "right"

                print(choice)
                print()
                self.postSignal(choice)


class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        blocks = {"left": pose.BlockLeft(),
                  "right": pose.BlockRight(),
                  "center": pose.Squat() # Does not get up after squatting
                  }
        for name in blocks:
            b = blocks[name]
            stand = pose.Stand()

            # Time less than 6 turns off robot after squatting
            #self.add_transition(blocker, S(name), b, T(3.25), blocker)
            self.add_transition(blocker, S(name), b, T(6.0), blocker)
