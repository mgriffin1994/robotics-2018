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
x_kp = 3e-3
x_ki = 0.0
x_kd = 3e-4
y_kp = 0.95
y_ki = 0.0
y_kd = 0.0
theta_kp = 0.75
theta_ki = 0.0
theta_kd = 1e-4

max_int = 10
moving_avg_samples = 3
max_num_frames = 30
max_int_steps = 10 # if want to use all steps, then set to float("inf")

x_error_thresh = 120 # w/in 12 cm of ball_distance_close behind ball
# y_error_thresh = 0.1
y_error_thresh = 0.12
theta_error_thresh = 0.075

### original thresholds for lab 4 kick checkout
# x_error_thresh = 120 # w/in 12 cm of ball_distance_close behind ball
# y_error_thresh = 0.1
# theta_error_thresh = 0.075

top_cam_width = 320
bot_cam_width = 320

# TODO: change these so it doesn't dribble the ball too well into the penalty box
kick_distance = 1400
slow_dribble_speed = 0.30

# ball_distance_close = 50
ball_distance_close = 20
ball_right_foot = 0.67

max_time_kick = 80

class ApproachBall(Node):

    def __init__(self):
        super(ApproachBall, self).__init__()
        self.x_errors = [x_error_thresh] * max_int # assume start out with at least x_error_thresh (and not below)
        self.y_errors = [y_error_thresh] * max_int
        self.theta_errors = [theta_error_thresh] * max_int
        self.prev_time = 1e-6
        self.num_frames_not_seen_ball = max_num_frames
        self.goal_distances = [kick_distance] * max_int
        self.prev_ball_distance = x_error_thresh + ball_distance_close
        self.prev_goal_centerx = 0
        self.prev_ball_centerx = 0
        self.ball_search_done = False
        self.avg_time_step = 0.01
        self.start_kick_frame = -1 # this variable is multipurpose (-1 means the kick has not been started, everything else means the frame number at which the kick started)
        self.kick_executed = False
        self.line_seen_counter = 0

    def run(self):

        if self.kick_executed == True and self.getFrames() - self.start_kick_frame > 10:
            return

        ### Grab the ball and goal world objects
        ball = mem_objects.world_objects[core.WO_BALL]
        goal = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]
        line = mem_objects.world_objects[core.WO_OWN_PENALTY]

        ### Tilt the head so we can see more
        commands.setHeadTilt(-18)

#         if (goal.seen):
#             print("goal seen")

        ### If we've seen the ball execute the PID loop
        if self.num_frames_not_seen_ball < max_num_frames:

            # print()
            # print("ball camera", ball.fromTopCamera)
            # print("ball seen", ball.seen)
            # print("ball distance", ball.visionDistance)
            # print("goal distance", goal.visionDistance)
            # print("goal seen", goal.seen)

            ### Compute ball and goal centers
            if ball.fromTopCamera:
                ball_center = ball.imageCenterX / top_cam_width if ball.seen else self.prev_ball_centerx
            else:
                ball_center = ball.imageCenterX / bot_cam_width if ball.seen else self.prev_ball_centerx
            goal_center = goal.imageCenterX / top_cam_width if goal.seen else self.prev_goal_centerx

            ### Compute errors
            # It is plus one for [-moving_avg_samples+1:] because we use the current measurement along with the last N-1 measurements for computing the average ... silly but whatever
            x_error = (ball.visionDistance - ball_distance_close) if ball.seen else (self.prev_ball_distance - ball_distance_close)
            x_error_avg = (x_error + sum(self.x_errors[-moving_avg_samples+1:])) / moving_avg_samples
            y_error = (goal_center - ball_center)
            y_error_avg = (y_error + sum(self.y_errors[-moving_avg_samples+1:])) / moving_avg_samples
            theta_error = (ball_right_foot - ball_center) if ball.seen else (ball_right_foot - self.prev_ball_centerx)
            theta_error_avg = (theta_error + sum(self.theta_errors[-moving_avg_samples+1:])) / moving_avg_samples

            ### Compute average distance to the goal
            goal_distance_avg = (goal.visionDistance + sum(self.goal_distances[-moving_avg_samples+1:])) / moving_avg_samples

            if line.seen:
                print('line seen but no kick yet')
                self.line_seen_counter += 1

        if line.seen and self.line_seen_counter > 5 and abs(x_error_avg) < x_error_thresh and abs(theta_error_avg) < theta_error_thresh:
            print('SAW LINE and in range and ball aligned. STOPPED')
            commands.kick()
            kick_executed = True
            self.start_kick_frame = self.getFrames()
            return

            # TODO: another measure to detect when to stop if got too close to line


            #if goal.seen and goal_distance_avg < 1200:
            #    print('TOO CLOSE BRO !!!!!')
            #    # TODO: just make him stop?
            #    return

            print('================')
#             print("x average error: ", x_error_avg)
#             print("y average error: ", y_error_avg)
#             print("theta average error: ", theta_error_avg)
            print('goal_distance_avg: %f' % (goal_distance_avg))

            ### Add to previous errors
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

            ### Keep track of previous ball and goal info
            self.prev_ball_distance = x_error + ball_distance_close
            self.prev_ball_centerx = ball_center if ball.seen else self.prev_ball_centerx
            self.prev_goal_centerx = goal_center if goal.seen else self.prev_goal_centerx

            ### Get the time
            time = self.getTime()
            print('time: %f' % (time))

            ### Last resort: just kick the ball if time is running out TODO
            if time > max_time_kick:
                print('---------- TIME RAN OUT ----------')
                commands.kick()
                kick_executed = True
                self.start_kick_frame = self.getFrames()
                return

            ### Compute velocities
            prev_x_avg = sum(self.x_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            x_vel = x_kp*x_error_avg + (x_kd*(x_error_avg - prev_x_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) + x_ki*(max(x_error + sum(self.x_errors), max_int))
            if x_vel > 0.35:
                x_vel = 0.35
            prev_y_avg = sum(self.y_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            y_vel = y_kp*y_error_avg + (y_kd*(y_error_avg - prev_y_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) + y_ki*(max(y_error + sum(self.y_errors), max_int))
            prev_theta_avg = sum(self.theta_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            theta_vel = theta_kp*theta_error_avg + (theta_kd*(theta_error_avg - prev_theta_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) + theta_ki*(max(theta_error + sum(self.theta_errors), max_int))

            ### If the kick has been started but no longer running, reset the flag to indicate the kick has not been started.
            ### We need this to make sure we don't try to keep kicking over and over again
            if self.start_kick_frame != -1:
                if not memory.kick_request.kick_running_ and self.getFrames() - self.start_kick_frame > 10:
                    print('Done kicking')
                    self.start_kick_frame = -1

            ### Close to the ball and aligned with goal and kick not started
            elif (abs(x_error_avg) < x_error_thresh and abs(y_error_avg) < y_error_thresh and abs(theta_error_avg) < theta_error_thresh):
                if goal_distance_avg > kick_distance:
                    print('Dribbling')
                    commands.setWalkVelocity(slow_dribble_speed, 0, 0)
                else:
                    print('Kicking')
                    print('----------> distance to goal: %d' % (goal_distance_avg))
                    self.kick_executed = True
                    commands.kick()
                self.start_kick_frame = self.getFrames() if self.start_kick_frame == -1 else self.start_kick_frame

            ### Too far and kick not started so use PID to walk toward the ball
            else:
#                 print('Walk toward ball')
#                 if (abs(x_error_avg) > x_error_thresh):
#                     print('x_error_avg too high: %f' % (x_error_avg))
#                 if (abs(y_error_avg) > y_error_thresh):
#                     print('y_error_avg too high: %f' % (y_error_avg))
#                 if (abs(theta_error_avg) > theta_error_thresh):
#                     print('theta_error_avg too high: %f' % (theta_error_avg))

                if goal_distance_avg < kick_distance and x_vel > slow_dribble_speed:
#                     print('walk toward ball and w/i kick distance')
                    commands.setWalkVelocity(slow_dribble_speed, y_vel, theta_vel)
                else:
                    commands.setWalkVelocity(x_vel, y_vel, theta_vel)

            # set previous time
            self.prev_time = time

        ### If we haven't seen the ball for max_num_frames
        else:
            # Start scanning by turning in place
            if not self.ball_search_done:
                print('Scan for ball')
                if goal.seen:
                    self.prev_goal_centerx = goal.imageCenterX / top_cam_width
                if ball.seen:
                    self.prev_ball_centerx = ball.imageCenterX / bot_cam_width if not ball.fromTopCamera else ball.imageCenterX / top_cam_width
                    self.prev_ball_distance = ball.visionDistance
                    self.ball_search_done = True
                if not self.ball_search_done:
                    direction = (ball_right_foot - self.prev_ball_centerx) / abs(ball_right_foot - self.prev_ball_centerx) # TODO: fix slow walk around one side of the ball
                    commands.setWalkVelocity(0, 0, direction* math.pi / 3)
            # Stop scanning and go back into PID loop
            else:
                print('Done scanning')
                self.num_frames_not_seen_ball = 0
                self.ball_search_done = False

        ### Increment the number of frames we haven't seen the ball
        if not ball.seen:
            self.num_frames_not_seen_ball += 1

class Ready(Task):
    def run(self):
        commands.setStiffness()
        commands.stand()
        if self.getTime() > 2.0:
            self.finish()

class Playing(LoopingStateMachine):
    def setup(self):
        ball_approach = ApproachBall()
        self.add_transition(ball_approach) #call ball_approach.run() every time_delay seconds (time_delay seconds between changes of control)
