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
from memory import joint_commands, localization_mem, robot_state, game_state
from task import Task
import UTdebug
import math
import numpy as np

predict_secs = 30 #TODO: make larger to be safe?
y_thresh = 550
time_delay = 1.0 / 30.
center_region = 125
x_thresh = -100 #100 mm in front of goal
num_samples = 10

vision_center_thresh = 50
rescan = 100 #every 2 secs at 100Hz

class Penalty(Node):

    def __init__(self):
        super(Penalty, self).__init__()

    def run(self):
        state = game_state

        print("I'm in bois!")

        # Test switch to Penalty Kick
        if not state.isPenaltyKick:
            self.postSignal("blocker")
            self.finish()

class Blocker(Node):

    def __init__(self):
        super(Blocker, self).__init__()
        self.data = []
        self.ball_not_seen_frames = 0
        self.bearing_samples = [0] * num_samples
        self.start_scan = -1
        self.ball_start_scan = -1
        self.cur_velocity = 0

    def run(self):
        commands.setStiffness()
        state = game_state

        # Test switch to Penalty Kick
        if state.isPenaltyKick:
            self.postSignal("penalty")
            self.finish()

        commands.setStiffness()
        commands.setHeadTilt(-15)

        ball = mem_objects.world_objects[core.WO_BALL]
        robot = mem_objects.world_objects[robot_state.WO_SELF]

#         beacon = mem_objects.world_objects[core.WO_BEACON_BLUE_YELLOW]
        # if ball.seen:
        #    z = np.array([ball.visionDistance, ball.visionBearing])
        #    self.data.append(z)
        #    mat = np.array(self.data)
        #    print(mat.shape)
        #    if len(self.data) > 2:
        #        print(np.cov(mat.T))
        #    print()

        #print(self.getFrames()

#'''
        rob_x = robot.loc.x
        rob_y = robot.loc.y
        #rob_x = -1250
        #rob_y = 0

        rob_t = robot.orientation
        ball_pos = localization_mem.getBallPosition()
        x_pos, y_pos = ball_pos.x, ball_pos.y
        ball_vel = localization_mem.getBallVel()

        #print()
        #print("=== python prints")
        #print("ball vel: ", ball.absVel.x, ", ", ball.absVel.y)
        #print("ball pos: ", ball.loc.x, ", ", ball.loc.y)

        x_vel, y_vel = ball_vel.x, ball_vel.y



        friction = 0.966

 #        if ball.seen:
 # #            #angle = np.arctan2((y_pos - rob_y), (x_pos - rob_x)) - rob_t
 #            commands.setHeadPan(ball.visionBearing, 0.1)
 #            self.ball_not_seen_frames = 0

 #        #TODO remove when done in sim?
 #        else:
 #           self.ball_not_seen_frames += 1


        #    #if self.ball_not_seen_frames > 50: #50 for sim, 100 for real?
        #    commands.setHeadPan(0, 0.1)
        #    #self.ball_not_seen_frames = 0

        #TODO: play with safe zone, rescan rate, speed of rescan
        #If ball safe distance away do periodic scan


        predicted_x = [x_pos + x_vel * time_delay * (1-friction**n)/(1-friction) for n in np.arange(0.0, predict_secs, 0.1)]
        predicted_y = [y_pos + y_vel * time_delay * (1-friction**n)/(1-friction) for n in np.arange(0.0, predict_secs, 0.1)]

        #print("predicted x: ", predicted_x[0], ", ",  predicted_x[-1])
        #print("predicted y: ", predicted_y[0], ", ",  predicted_y[-1])
        #print("robot x, y: ", rob_x, ", ", rob_y)
        #print("velocity x, y: ", x_vel, ", ", y_vel)

        if ball.seen and any(x <= -1500 - x_thresh for x in predicted_x):
            possible_goal_frames = [i for i, x in enumerate(predicted_x) if x <= -1500 - x_thresh]
            if any(abs(predicted_y[i] - 0) < y_thresh for i in possible_goal_frames):
                y_pred = predicted_y[possible_goal_frames[0]]
                print('num_frames', possible_goal_frames[0])
                choice = ""
                if abs(y_pred - rob_y) <= center_region:
                    choice = "center"
                elif y_pred >= 0:
                    choice = "left"
                elif y_pred < 0:
                    choice = "right"

                print(choice)
                print()
                if choice:
                    self.postSignal(choice)

        if ball.seen:
            self.bearing_samples.append(ball.visionBearing)
            bearing_avg = sum(self.bearing_samples) / num_samples
            # commands.setWalkVelocity(0.2, 0.0, bearing_avg) # TODO: put back
            self.bearing_samples.pop(0)
            #print("Vision Bearing: ", ball.visionBearing)
            #print("Avg Vision Bearing: ", bearing_avg)

        #TODO if ball too close stop walking to prepare for block?
        y_exp = (y_pos/(x_pos + 1750))*(rob_x + 1750) #between ball and center of goal

        y_diff = y_exp - rob_y
        theta_diff = rob_t
        #theta_diff = ball.imageCenterX / 320 - 0.5
        x_diff = rob_x - (-1500 + 325 + 350)

        keep_center_gain = 0.1
        #keep_center_gain = 0.0


        # commands.setWalkVelocity(-x_diff*keep_center_gain, y_diff*keep_center_gain, -theta_diff*0.2) # TODO: put bakc


# TODO: Put back
#        if x_pos - rob_x > 750 or not ball.seen:
#            if self.getFrames() % rescan == 0 or self.start_scan != -1:
#                #commands.setWalkVelocity(0, 0, 0)
#                self.start_scan = self.getFrames() if self.start_scan == -1 else self.start_scan
#                commands.setHeadPan(-math.pi / 5, 0.10)
#                if self.getFrames() - self.start_scan > 20: #0.5 sec at 100Hz (min 10)
#                    commands.setHeadPan(math.pi / 3, 0.4)
#                    if self.getFrames() - self.start_scan > 60: #0.5 sec at 100Hz
#                        commands.setHeadPan(0, 0.1)
#                        self.start_scan = -1
#        elif ball.seen:
#            commands.setHeadPan(ball.visionBearing, 0.1)

        # TODO: Add sit
        # if x_pos - rob_x < 250: # TODO: put back
            # commands.setWalkVelocity(0, 0, 0) # TODO: put back

        #TODO: add vision checks for lines to prevent walking too far

        #TODO play with timing of this ball find scan
        #If it's been 50 frames without seeing ball, do shorter scan
        #If see ball during this scan, stop scanning
        # if self.ball_not_seen_frames > 50:
        #     commands.setWalkVelocity(0, 0, 0)
        #     if self.getFrames() % rescan == 0 or self.ball_start_scan != -1:
        #         self.ball_start_scan = self.getFrames() if self.ball_start_scan == -1 else self.ball_start_scan
        #         commands.setHeadPan(-math.pi / 4, 0.1)
        #         if self.getFrames() - self.ball_start_scan > 25: #0.25 sec at 100Hz (min 10)
        #             commands.setHeadPan(math.pi / 4, 0.1)
        #             if self.getFrames() - self.ball_start_scan > 50: #0.25 sec at 100Hz
        #                 commands.setHeadPan(0, 0.1)
        #                 self.ball_start_scan = -1


        # if ball.seen and abs(rob_y - y_pos) > vision_center_thresh:

        # else:
        #     commands.setWalkVelocity(0, 0, theta_diff*0.275)


        #If too close to sides of box, don't move
        # if (y_diff < 0 and rob_y < -700 + 150):
        #     commands.setWalkVelocity(0, 0, theta_diff*0.275)

        # elif (y_diff > 0 and rob_y > 700 - 150):
        #     commands.setWalkVelocity(0, 0, theta_diff*0.275)


#             if line.seen:
#                 line_start = line.lowerHeight
#                 line_end = line.upperHeight
#
#                 # Value in pixels
#                 line_tolerance = 10
#
#                 if line_start < line_tolerance or line_end > line_end - line_tolerance:
#                     commands.setWalkVelocity(0.2, 0.0, bearing_avg)
#                     self.bearing_samples.pop(0)
#                     print("Vision Bearing: ", ball.visionBearing)
#                     print("Avg Vision Bearing: ", bearing_avg)
#                 else:
#                     print("CORNER OF GOAL BOX")
#             else:
#                 print("NO LINE")
#
#         #TODO: change init location of robot to be center of box?
# #         if abs(rob_y - y_pos) > vision_center_thresh:
# #             diff = y_pos - rob_y
# #
# #             #If too close to sides of box, don't move
# #
# #             if (diff > 0 and rob_y < -700 + 150):
# #                 commands.setWalkVelocity(0, 0, 0)
# #                 return
# #
# #             elif (diff < 0 and rob_y > 700 - 150):
# #                 commands.setWalkVelocity(0, 0, 0)
# #                 return
# #
# #             #print('diff: %d' % (diff))
# #             #print('diff * keep_center_gain: %f' % (diff * keep_center_gain))
# #             #if (diff < 0):
# #             #commands.setWalkVelocity(0.1, keep_center_gain * diff, 0)
# #             #else:
# #             #    commands.setWalkVelocity(0.1, keep_center_gain * diff, 0.05)
# #         else:
# #             commands.setWalkVelocity(0, 0, 0)



#'''

class Ready(Task):
    def run(self):
       commands.stand()
# TODO: put back
#        if self.getTime() > 3.0:
#            commands.setHeadPanTilt(-math.pi / 4, -15, 0.5)
#            if self.getTime() > 8.0:
#                commands.setHeadPan(math.pi / 4, 0.5)
#                if self.getTime() > 13.0:
#                    commands.setHeadPan(0, 0.5)


# TODO: How to switch between kicking and goalie?
class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        penalty = Penalty()

        blocks = {"left": pose.BlockLeft(),
                   "right": pose.BlockRight(),
                   "center": pose.SitBlock(),
                   }
        #blocks = {
        #    "center": pose.SitBlock()
        #}
        for name in blocks:
            b = blocks[name]
            self.add_transition(blocker, S(name), b, T(10.25), blocker)

        self.add_transition(blocker, S("penalty"), penalty)
        self.add_transition(penalty, S("blocker"), blocker)
