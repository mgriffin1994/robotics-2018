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
x_kp = 1e-3
x_ki = 0.0
x_kd = 1e-4
y_kp = 0.0
y_ki = 0.0
y_kd = 0.0
theta_kp = 0.5
theta_ki = 0.0
theta_kd = 1e-4

max_length = 10
max_int = 10
moving_avg_samples = 3
max_num_frames = 30
max_distance = 2000
max_int_steps = 10 #if want to use all steps, then set to float("inf")
ball_distance_close = 50

x_error_thresh = 10 #w/in 1 cm of ball_distance_close behind ball
y_error_thresh = float('inf') #10 change to actual value when testing this
theta_error_thresh = math.pi / 10 #change to actual value when testing this

kick_distance = 500 #half meter kick (should be able to do 1 meter kick for extra credit)


class ApproachBall(Node):

    def __init__(self):
        super(ApproachBall, self).__init__()
        self.x_errors = [x_error_thresh] * max_int #assume start out with at least x_error_thresh (and not below)
        self.y_errors = [y_error_thresh] * max_int
        self.theta_errors = [theta_error_thresh] * max_int
        self.prev_time = 1e-6
        self.num_frames_not_seen_ball = max_num_frames
        self.prev_ball_distance = x_error_thresh + ball_distance_close
        self.prev_ball_bearing = -math.pi / 2
        self.prev_goal_centerx = 0
        self.prev_ball_centerx = 0
        self.goal_search_done = False
        self.ball_search_done = False
        self.avg_time_step = 0.01

    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        goal = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]

        # if self.num_frames_not_seen_ball < max_num_frames:
        if True:
           
            print()
            print("ball camera", ball.fromTopCamera)
            print("ball seen", ball.seen)
            print("ball distance", ball.visionDistance)
            #print("goal seen", goal.seen)


            ###Compute errors
            x_error = ball.visionDistance - ball_distance_close if ball.seen else self.prev_ball_distance - ball_distance_close
            x_error_avg = (x_error + sum(self.x_errors[-moving_avg_samples+1:])) / moving_avg_samples

            ball_center = ball.imageCenterX if ball.seen else self.prev_ball_centerx
            goal_center = goal.imageCenterX if goal.seen else self.prev_goal_centerx

            y_error = ball_center - goal_center
            y_error_avg = (y_error + sum(self.y_errors[-moving_avg_samples+1:])) / moving_avg_samples


            theta_error = ball.visionBearing if ball.seen else self.prev_ball_bearing
            theta_error_avg = (theta_error + sum(self.theta_errors[-moving_avg_samples+1:])) / moving_avg_samples

            print("x error", x_error, "x average error", x_error_avg)
            #print("y error", y_error, "y average error", y_error_avg)
            # print("theta error", theta_error, "theta average error", theta_error_avg)

            ###Try to score goal if aligned
            #if close enough to ball, aimed at ball, and aimed at goal try to dribble it forward
            # otherwise continue with below and try to realign
            #if abs(x_error_avg) < x_error_thresh and abs(y_error_avg) < y_error_thresh and abs(theta_error_avg) < theta_error_thresh:
            #    #self.finish()
            #    if goal.visionDistance > kick_distance: 
            #        #if aligned with ball and goal and far away walk forward
            #        commands.setWalkVelocity(1.0, 0, 0) 
            #    else:
            #        commands.kick() #if close enough to goal and aligned try kick
            #        #what next
            #    return

           ###Compute velocities
            time = self.getTime()

            prev_x_avg = sum(self.x_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            x_vel = x_kp*x_error_avg + (x_kd*((x_error_avg - prev_x_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) if len(self.x_errors) > 1 else 0.0) + x_ki*(max(x_error + sum(self.x_errors), max_int))

            if ball.visionDistance > 1500 or (self.prev_ball_distance > 1500 and not ball.seen):
                x_vel = 1.0


            prev_y_avg = sum(self.y_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            y_vel = y_kp*y_error_avg + (y_kd*((y_error_avg - prev_y_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step))))if len(self.y_errors) > 1 else 0.0) + y_ki*(max(y_error + sum(self.y_errors), max_int))
            

            prev_theta_avg = sum(self.theta_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            theta_vel = theta_kp*theta_error_avg + (theta_kd*((theta_error_avg - prev_theta_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) if len(self.theta_errors) > 1 else 0.0) + theta_ki*(max(theta_error + sum(self.theta_errors), max_int))

            # print("x velocity", x_vel)
            #print("y velocity", y_vel)
            # print("theta velocity", y_vel)

            ###Add to previous errors

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

            self.prev_ball_distance = x_error + ball_distance_close
            self.prev_ball_bearing = theta_error
            self.prev_ball_centerx = ball.imageCenterX if ball.seen else self.prev_ball_centerx
            self.prev_goal_centerx = goal.imageCenterX if goal.seen else self.prev_goal_centerx

            ###Start walking via computed velocities
            commands.setWalkVelocity(x_vel, y_vel, theta_vel)

        #else:
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
            # commands.setWalkVelocity(0, 0, 0)
        if not ball.seen:
            self.num_frames_not_seen_ball += 1

#TODO: do we want to make sure head is tracking ball?
        
class Ready(Task):
    def run(self):
        commands.setStiffness()
        commands.stand()
        if self.getTime() > 2.0:
            self.finish()

class Playing(LoopingStateMachine):
    def setup(self):
        ball_approach = ApproachBall()

        #call ball_approach.run() every time_delay seconds (time_delay seconds between changes of control)
        self.add_transition(ball_approach, T(time_delay), ball_approach)
