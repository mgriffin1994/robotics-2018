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

# this is for the hacky version of keeping the ball in center of view
vision_center_thresh = 40
keep_center_gain = 0.007
image_center_x = 160
image_center_y = 120

# TODO:
#   - Move to center of penalty box using localization/vision
#   - Test different blocks
#       - Develop different squat block, the one in pose.py sucks
#   - Use Kalman filter to predict ball location and move, if necessary
#       - Speed up prediction, blocking is too slow
#   - Choose block to use (may need more than the three provided)

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
#         ball_x, ball_y = ball_pos.x, ball_pos.y # particle filter's estimate of ball's (x,y)
        ball_x, ball_y = ball.imageCenterX, ball.imageCenterY # vision estimate of ball's (x,y)
        ball_vel = localization_mem.getBallVel()

#         print()
#         print("=== python prints")
#         print("ball vel: ", ball.absVel.x, ", ", ball.absVel.y)
#         print("ball pos: ", ball.loc.x, ", ", ball.loc.y)

        ball_xvel, ball_yvel = ball_vel.x, ball_vel.y

        ### turn head toward the ball --> doesn't work
        #if ball.seen:
        #    angle = np.arctan((ball_y - rob_y)/(ball_x - rob_x + 1e-5))
        #    commands.setHeadPan(angle, 0.1)
        #else:
        #    commands.setHeadPan(0, 0.1)

        predicted_x = [ball_x + ball_xvel * time_delay * n for n in np.arange(0.0, predict_secs, 0.1)]
        predicted_y = [ball_y + ball_yvel * time_delay * n for n in np.arange(0.0, predict_secs, 0.1)]

#         print("predicted x: ", predicted_x[0], ", ",  predicted_x[-1])
#         print("predicted y: ", predicted_y[0], ", ",  predicted_y[-1])
#         print("robot x, y: ", rob_x, ", ", rob_y)
#         print("velocity x, y: ", ball_xvel, ", ", ball_yvel)

#         print('======')
#         print('ball_x: %d ball_y: %d' % (ball_x, ball_y))
#         print('rob_x:  %d rob_y:  %d' % (rob_x, rob_y))

        ### keep the ball in the center of vision
        # vision bounds in pixels (x: 320 and y: 240)
#         if abs(ball_x - image_center_x) > vision_center_thresh:
#             diff = image_center_x - ball_x
#             print('diff: %d' % (diff))
#             print('diff * keep_center_gain: %f' % (diff * keep_center_gain))
#             if (diff < 0):
#                 commands.setWalkVelocity(0.1, keep_center_gain * diff, 0)
#             else:
#                 commands.setWalkVelocity(0.1, keep_center_gain * diff, 0.05)
#         else:
#             commands.setWalkVelocity(0, 0, 0)

        ### execute a block if the ball is predicted to have been shot into the goal
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
