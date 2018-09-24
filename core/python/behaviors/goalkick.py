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
import math
from task import Task

time_delay = 0.1
x_kp = 0.001
x_ki = 0.0
x_kd = 0.0
y_kp = 0.0
y_ki = 0.0
y_kd = 0.0
theta_kp = 0.0
theta_ki = 0.0
theta_kd = 0.0

max_length = 10
max_int = 10
max_num_frames = 30
max_distance = 2000
max_int_steps = 10 #if want to use all steps, then set to float("inf")
ball_distance_close = 200

x_error_thresh = 10
y_error_thresh = float('inf') #10 change to actual value when testing this
theta_error_thresh = float('inf') #math.pi / 10 change to actual value when testing this


class ApproachBall(Node):

    def __init__(self):
        super(ApproachBall, self).__init__()
        self.x_errors = [750] * 10
        self.y_errors = []
        self.theta_errors = []
        self.prev_time = 0.0
        self.num_frames_not_seen_ball = max_num_frames
        self.prev_ball_distance = max_distance
        self.prev_ball_bearing = -math.pi / 2
        self.prev_goal_centerx = 0
        self.prev_ball_centerx = 0
        self.goal_search_done = False
        self.ball_search_done = False


    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        goal = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]

        #if self.num_frames_not_seen_ball < max_num_frames:
        if True:

            x_error = ball.visionDistance if ball.seen else self.prev_ball_distance
            x_error = (x_error + sum(self.x_errors[8:])) / 3
            ball_center = ball.imageCenterX if ball.seen else self.prev_ball_centerx
            goal_center = goal.imageCenterX if goal.seen else self.prev_goal_centerx

            x_error -= ball_distance_close

            print("x_error: {}".format(x_error))
            print(self.x_errors)
            print("ball seen", ball.seen)

            y_error = ball_center - goal_center
            theta_error = ball.visionBearing if ball.seen else self.prev_ball_bearing

            if abs(x_error) < x_error_thresh and abs(y_error) < y_error_thresh and abs(theta_error) < theta_error_thresh:
                self.finish()

            #We probably don't really need an I control since there will never really be anything preventing us from getting to the desired place (unless someone is holding the robot in place) and we have a max speed
            # D control will help slow down and not oscillate towards the desired place

            time = self.getTime()

            # TODO: Fix this eventually
            if time == 0.0:
                time = 1.0

            print(time, self.prev_time)
            x_vel = x_kp*x_error + (x_kd*((x_error - self.x_errors[-2]) / (2*(time - self.prev_time))) if len(self.x_errors) > 1 else 0.0) + x_ki*(max(sum(self.x_errors), max_int))

            x_error += ball_distance_close

            if ball.visionDistance > 1500 or (self.prev_ball_distance > 1500 and not ball.seen):
                x_vel = 1.0
            print("x velocity", x_vel)

            y_vel = y_kp*y_error + (y_kd*((y_error - self.y_errors[-2]) / (2*(time - self.prev_time))) if len(self.y_errors) > 1 else 0.0) + y_ki*(max(sum(self.y_errors), max_int))
            theta_vel = theta_kp*theta_error + (theta_kd*((theta_error - self.theta_errors[-2]) / (2*(time - self.prev_time))) if len(self.theta_errors) > 1 else 0.0) + theta_ki*(max(sum(self.theta_errors), max_int))

            self.prev_time = time


            self.x_errors.append(x_error)
            self.y_errors.append(y_error)
            self.theta_errors.append(theta_error)
            if len(self.x_errors) > max_int_steps:
                self.x_errors.pop(0)
            if len(self.y_errors) > max_int_steps:
                self.y_errors.pop(0)
            if len(self.theta_errors) > max_int_steps:
                self.theta_errors.pop(0)

            commands.setWalkVelocity(x_vel, y_vel, theta_vel)
            self.prev_ball_distance = x_error
            self.prev_ball_bearing = theta_error
            self.prev_ball_centerx = ball.imageCenterX if ball.seen else self.prev_ball_centerx
            self.prev_goal_centerx = goal.imageCenterX if goal.seen else self.prev_goal_centerx

#        else:
#            if not self.goal_search_done and not self.ball_search_done:
#                commands.setWalkVelocity(0, 0, math.pi / 3)
#                    self.goal_search_done = True
#                    self.prev_goal_centerx = goal.imageCenterX
#                if ball.seen:
#                    self.prev_ball_centerx = ball.imageCenterX
#                    self.prev_ball_bearing = ball.visionBearing
#                    self.prev_ball_distance = ball.visionDistance
#                #search for goal - set self.goal_search_done = True once find goal (and set prev values)
#            if self.goal_search_done and not self.ball_search_done:
#                direction = self.prev_ball_bearing / abs(self.prev_ball_bearing)
#                commands.setWalkVelocity(0, 0, direction * math.pi / 3)
#                if goal.seen:
#                    self.prev_goal_centerx = goal.imageCenterX
#                if ball.seen:
#                    self.prev_ball_centerx = ball.imageCenterX
#                    self.prev_ball_bearning = ball.visionBearing
#                    self.prev_ball_distance = ball.visionDistance
#                    self.ball_search_done = True
#                #search for ball - set self.ball_search_done = True once find ball (and set prev values)
#            if self.goal_search_done and self.ball_search_done:
#                self.num_frames_not_seen_ball = 0
#                self.goal_search_done = False
#                self.ball_search_done = False
#        if not ball.seen:
#            self.num_frames_not_seen_ball += 1

#TODO: do we want to make sure head is tracking ball?

#class ScoreGoal(Node):
#    def run(self):
#        commands.S

class Ready(Task):
    def run(self):
        commands.setStiffness()
        commands.stand()
        if self.getTime() > 2.0:
            self.finish()

class Playing(LoopingStateMachine):
    def setup(self):
        sit = pose.Sit() #TODO replace with goal scoring Node later
        ball_approach = ApproachBall()

        self.add_transition(ball_approach, C, sit)
