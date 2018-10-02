"""Simple keeper behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

from time import sleep
import core
import pose
import cfgstiff
import commands
import mem_objects
from state_machine import Node, S, T, C, LoopingStateMachine, StateMachine
import UTdebug
import math
from task import Task
import memory

time_delay = 0.1
x_kp = 2e-3
x_ki = 0.0
x_kd = 3e-4
y_kp = 0.85
y_ki = 0.0
y_kd = 0.0
theta_kp = 0.75
theta_ki = 0.0
theta_kd = 1e-4

max_length = 10
max_int = 10
moving_avg_samples = 3
max_num_frames = 30
max_int_steps = 10 #if want to use all steps, then set to float("inf")

ball_distance_close = 50
ball_right_foot = 0.67

x_error_thresh = 120 #w/in 12 cm of ball_distance_close behind ball
y_error_thresh = 0.1
theta_error_thresh = 0.075

vel_thresh = 0.1

kick_distance = 1600 #half meter kick (should be able to do 1 meter kick for extra credit), also goal.visionDistance is to back of goal not front of goal

top_cam_width = 320
bot_cam_width = 320


class ApproachBall(Node):

    def __init__(self):
        super(ApproachBall, self).__init__()
        self.x_errors = [x_error_thresh] * max_int #assume start out with at least x_error_thresh (and not below)
        self.y_errors = [y_error_thresh] * max_int
        self.theta_errors = [theta_error_thresh] * max_int
        self.prev_time = 1e-6
        self.num_frames_not_seen_ball = max_num_frames
        self.goal_distances = [kick_distance] * max_int
        self.prev_ball_distance = x_error_thresh + ball_distance_close
        self.prev_goal_centerx = 0
        self.prev_ball_centerx = 0
        self.goal_search_done = False
        self.ball_search_done = False
        self.start_driving = False
        self.avg_time_step = 0.01
        self.start_kick_frame = -1
        self.kicking = False

    def run(self):
        ball = mem_objects.world_objects[core.WO_BALL]
        goal = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]

        if self.num_frames_not_seen_ball < max_num_frames:

            #print()
            #print("ball camera", ball.fromTopCamera)
            #print("ball seen", ball.seen)
            #print("ball distance", ball.visionDistance)
            #print("goal distance", goal.visionDistance)
            #print("goal seen", goal.seen)


            ###Compute errors
            x_error = (ball.visionDistance - ball_distance_close) if ball.seen else (self.prev_ball_distance - ball_distance_close)
            x_error_avg = (x_error + sum(self.x_errors[-moving_avg_samples+1:])) / moving_avg_samples
            goal_distance_avg = (goal.visionDistance + sum(self.goal_distances[-moving_avg_samples+1:])) / moving_avg_samples

            if ball.fromTopCamera:
                ball_center = ball.imageCenterX / top_cam_width if ball.seen else self.prev_ball_centerx
            else:
                ball_center = ball.imageCenterX / bot_cam_width if ball.seen else self.prev_ball_centerx

            goal_center = goal.imageCenterX / top_cam_width if goal.seen else self.prev_goal_centerx

            y_error = (goal_center - ball_center)
            y_error_avg = (y_error + sum(self.y_errors[-moving_avg_samples+1:])) / moving_avg_samples


            theta_error = (ball_right_foot - ball_center) if ball.seen else (ball_right_foot - self.prev_ball_centerx)
            theta_error_avg = (theta_error + sum(self.theta_errors[-moving_avg_samples+1:])) / moving_avg_samples

            #print("ball_center ", ball_center, " goal_center ", goal_center)
            #print("x error", x_error, "   |   ", "x average error", x_error_avg)
            #print("y error", y_error, "   |   ", "y average error", y_error_avg)
            #print("theta error", theta_error, "   |   ", "theta average error", theta_error_avg)


            ###Add to previous errors


            self.x_errors.append(x_error)
            self.y_errors.append(y_error)
            self.theta_errors.append(theta_error)
            self.goal_distances.append(goal.visionDistance)
            if len(self.x_errors) > max_int_steps:
                self.x_errors.pop(0)
            if len(self.y_errors) > max_int_steps:
                self.y_errors.pop(0)
            if len(self.theta_errors) > max_int_steps:
                self.theta_errors.pop(0)
            if len(self.goal_distances) > max_int_steps:
                self.goal_distances.pop(0)

            self.prev_ball_distance = x_error + ball_distance_close
            self.prev_ball_centerx = ball_center if ball.seen else self.prev_ball_centerx
            self.prev_goal_centerx = goal_center if goal.seen else self.prev_goal_centerx

            time = self.getTime()

            ##Compute velocities
            prev_x_avg = sum(self.x_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            x_vel = x_kp*x_error_avg + (x_kd*(x_error_avg - prev_x_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) + x_ki*(max(x_error + sum(self.x_errors), max_int))

            if ball.visionDistance > 1500 or (self.prev_ball_distance > 1500 and not ball.seen):
                x_vel = 1.0

            prev_y_avg = sum(self.y_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            y_vel = y_kp*y_error_avg + (y_kd*(y_error_avg - prev_y_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) + y_ki*(max(y_error + sum(self.y_errors), max_int))


            prev_theta_avg = sum(self.theta_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            theta_vel = theta_kp*theta_error_avg + (theta_kd*(theta_error_avg - prev_theta_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) + theta_ki*(max(theta_error + sum(self.theta_errors), max_int))
            #theta_vel = sgn(theta_vel)*max(abs(theta_vel), 0.35)

            #print("x velocity", x_vel, x_error_avg, (x_error_avg - prev_x_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step))))
            #print("y velocity", y_vel)
            #print("theta velocity", y_vel)


            ###Try to score goal if aligned, otherwise walk towards being aligned

            #if close enough to ball, aimed at ball, and aimed at goal try to dribble it forward
            # otherwise continue with below and try to realign

            if self.start_kick_frame != -1:
                #print('kicking')
                #print(memory.kick_request.kick_running_, self.getFrames() - self.start_kick_frame)
                if not memory.kick_request.kick_running_ and self.getFrames() - self.start_kick_frame > 10:
                    #self.kicking = False
                    self.start_kick_frame = -1
                    #print('done kicking')

            elif (abs(x_error_avg) < x_error_thresh and abs(y_error_avg) < y_error_thresh and abs(theta_error_avg) < theta_error_thresh):
            #if ((abs(x_vel) < vel_thresh and abs(y_vel) < vel_thresh and abs(theta_vel) < vel_thresh) or (self.start_kick_frame != -1)):
                print("Within threshold")
                

                #if goal_distance_avg > kick_distance:
                    #print('driving')
                    #self.start_driving = True
                    #if aligned with ball and goal and far away walk forward
                #    commands.setWalkVelocity(0.75, 0, 0)
                    #then if become unaligned again next time switch back to pid (until close enough to goal)
                



                #else:
                    # commands.setWalkVelocity(0, 0, 0) #continuing to drive even within kick_distance???
                    #self.start_kick_frame = 1 #TODO remove, while testing once get aligned and within kick_distance stand still forever
                    #self.start_driving = False

                    #    #if close enough to goal and either aligned or have started kicking already (to make sure kick gets called) run this code
                #    #TODO see if just running commands.kick() is fine


                #    #sleep(.3)
                    #self.kicking = True
                #if self.getFrames() - self.start_kick_frame <= 3:
                if self.start_kick_frame == -1:
                    commands.kick()
                    print("One kick to rule them all")
                self.start_kick_frame = self.getFrames() if self.start_kick_frame == -1 else self.start_kick_frame



                    #self.start_kick_frame = self.getFrames() if self.start_kick_frame == -1 else self.start_kick_frame       
                    ##if self.getFrames() - self.start_kick_frame <= 50:
                    ##    print("Pause for balance")
                    ##    commands.setWalkVelocity(0.0, 0, 0)
                    #if self.getFrames() - self.start_kick_frame <= 3: 
                    #    print("KICK!")
                    #    commands.kick() #if close enough to goal and aligned try kick
                    #elif self.getFrames() - self.start_kick_frame > 10 and not memory.kick_request.kick_running_:
                    #    print("Done kick")
                    #    self.start_kick_frame = -1

            else:
                #print('PIDing')
                ##Start walking
                #never approach the ball when within kick_distance, only kick
                #if goal.visionDistance > kick_distance:
                commands.setWalkVelocity(x_vel, y_vel, theta_vel)
        
            self.prev_time = time
        
        else:
            #print('starting scan')
            if not self.ball_search_done:
                if goal.seen:
                    self.prev_goal_centerx = goal.imageCenterX / top_cam_width
                if ball.seen:
                    self.prev_ball_centerx = ball.imageCenterX / bot_cam_width if not ball.fromTopCamera else ball.imageCenterX / top_cam_width
                    self.prev_ball_bearning = ball.visionBearing
                    self.prev_ball_distance = ball.visionDistance
                    self.ball_search_done = True
                    print('done searching')
                if not self.ball_search_done:
                    direction = (ball_right_foot - self.prev_ball_centerx) / abs(ball_right_foot - self.prev_ball_centerx)
                    commands.setWalkVelocity(0, 0, direction* math.pi / 3)
                #search for ball - set self.ball_search_done = True once find ball (and set prev values)
            else:
                self.num_frames_not_seen_ball = 0
                self.ball_search_done = False
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
        self.add_transition(ball_approach)
