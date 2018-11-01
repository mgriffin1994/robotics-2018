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
import memory
from task import Task
import UTdebug
import math
import numpy as np

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

top_cam_width = 320
bot_cam_width = 320

# TODO: change these so it doesn't dribble the ball too well into the penalty box
kick_distance = 1400
slow_dribble_speed = 0.30

ball_distance_close = 20 #50
ball_right_foot = 0.67

max_time_kick = 85

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
        state = game_state

        # Test switch to Penalty Kick
        if not state.isPenaltyKick:
            self.postSignal("blocker")
            self.finish()

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











predict_secs = 30 #TODO: make larger to be safe?
y_thresh = 550
time_delay_b = 1.0 / 30.
center_region = 125
x_thresh = -100 #100 mm in front of goal
num_samples = 10

#vision_center_thresh = 50
rescan = 100 #every 2 secs at 100Hz

vision_center_thresh = 40
keep_center_gain = 0.007
image_center_x = 160
image_center_y = 120

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




        #Kalman based goal blocking
        # predicted_x = [x_pos + x_vel * time_delay_b * (1-friction**n)/(1-friction) for n in np.arange(0.0, predict_secs, 0.1)]
        # predicted_y = [y_pos + y_vel * time_delay_b * (1-friction**n)/(1-friction) for n in np.arange(0.0, predict_secs, 0.1)]

        # print("predicted x: ", predicted_x[0], ", ",  predicted_x[-1])
        # print("predicted y: ", predicted_y[0], ", ",  predicted_y[-1])
        # print("robot x, y: ", rob_x, ", ", rob_y)
        # print("velocity x, y: ", x_vel, ", ", y_vel)

        # if ball.seen and any(x <= -1500 - x_thresh for x in predicted_x):
        #     possible_goal_frames = [i for i, x in enumerate(predicted_x) if x <= -1500 - x_thresh]
        #     if any(abs(predicted_y[i] - 0) < y_thresh for i in possible_goal_frames):
        #         y_pred = predicted_y[possible_goal_frames[0]]
        #         print('num_frames', possible_goal_frames[0])
        #         choice = ""
        #         if abs(y_pred - rob_y) <= center_region:
        #             choice = "center"
        #         elif y_pred >= 0:
        #             choice = "left"
        #         elif y_pred < 0:
        #             choice = "right"

        #         print(choice)
        #         print()
        #         if choice:
        #             self.postSignal(choice)

        if not ball.fromTopCamera and ball.imageCenterY > 40:
            if ball.imageCenterX < 160:
                self.postSignal('left')
            #elif ball.imageCenterX > 320 - 140:
            #    self.postSignal('right')
            else: 
                #self.postSignal('right')
                self.postSignal('left')



        #Rotating to block ball
        # if ball.seen:
        #     self.bearing_samples.append(ball.visionBearing)
        #     bearing_avg = sum(self.bearing_samples) / num_samples
        #     # commands.setWalkVelocity(0.2, 0.0, bearing_avg) # TODO: put back
        #     self.bearing_samples.pop(0)
        #     #print("Vision Bearing: ", ball.visionBearing)
        #     #print("Avg Vision Bearing: ", bearing_avg)




        #TODO if ball too close stop walking to prepare for block?
        #y_exp = (y_pos/(x_pos + 1750))*(rob_x + 1750) #between ball and center of goal

        #y_diff = y_exp - rob_y
        #y_diff = ball.imageCenterX / 320 - 0.5
        #theta_diff = rob_t
        #theta_diff = ball.imageCenterX / 320 - 0.5
        #x_diff = rob_x - (-1500 + 325 + 350)

        keep_center_gain = 0.1 #0.0



        # commands.setWalkVelocity(-x_diff*keep_center_gain, y_diff*keep_center_gain, -theta_diff*0.2) # TODO: put bakc
        #commands.setWalkVelocity(0.2, y_diff*keep_center_gain, np.sign(y_diff)*0.05)


        if (ball.fromTopCamera or (not ball.fromTopCamera and ball.imageCenterY < 20)) and abs(ball.imageCenterX - image_center_x) > vision_center_thresh:
            diff = image_center_x - ball.imageCenterX
            print('diff: %d' % (diff))
            print('diff * keep_center_gain: %f' % (diff * keep_center_gain))
            #RIGHT
            if (diff < 0):
                commands.setWalkVelocity(0.2, keep_center_gain * diff, -0.03)
            #LEFT
            else:
                commands.setWalkVelocity(0.2, keep_center_gain * diff, 0.05)
        else:
            commands.setWalkVelocity(0, 0, 0)





        #TODO: play with safe zone, rescan rate, speed of rescan
        #If ball safe distance away do periodic scan
        # if x_pos - rob_x > 750 or not ball.seen:
        #     if  self.getFrames() % rescan == 0 or self.start_scan != -1:
        #         #commands.setWalkVelocity(0, 0, 0)
        #         self.start_scan = self.getFrames() if self.start_scan == -1 else self.start_scan
        #         commands.setHeadPan(-math.pi / 5, 0.10)
        #         if self.getFrames() - self.start_scan > 20: #0.5 sec at 100Hz (min 10)
        #             commands.setHeadPan(math.pi / 3, 0.4)
        #             if self.getFrames() - self.start_scan > 60: #0.5 sec at 100Hz
        #                 commands.setHeadPan(0, 0.1)
        #                 self.start_scan = -1
        


        #if ball.seen:
        #    commands.setHeadPan(ball.visionBearing, 0.1)




        # TODO: Add sit
        # if x_pos - rob_x < 250: # TODO: put back
            # commands.setWalkVelocity(0, 0, 0) # TODO: put back

        #TODO: add vision checks for lines to prevent walking too far

        # if line.seen:
        #     line_start = line.lowerHeight
        #     line_end = line.upperHeight

        #     # Value in pixels
        #     line_tolerance = 10

        #     if line_start < line_tolerance or line_end > line_end - line_tolerance:
        #         commands.setWalkVelocity(0.2, 0.0, bearing_avg)
        #         self.bearing_samples.pop(0)
        #         print("Vision Bearing: ", ball.visionBearing)
        #         print("Avg Vision Bearing: ", bearing_avg)
        #     else:
        #         print("CORNER OF GOAL BOX")
        # else:
        #     print("NO LINE")





class Ready(Task):
    def run(self):
        commands.setStiffness()
        commands.stand()
# TODO: put back
        #if self.getTime() > 3.0:
            # commands.setHeadPanTilt(-math.pi / 4, -15, 0.5)
            # if self.getTime() > 8.0:
            #     commands.setHeadPan(math.pi / 4, 0.5)
            #     if self.getTime() > 13.0:
            #         commands.setHeadPan(0, 0.5)


# TODO: How to switch between kicking and goalie?
class Playing(LoopingStateMachine):
    def setup(self):
        blocker = Blocker()
        penalty = ApproachBall()

        blocks = {"left": pose.BlockLeft(),
                   "right": pose.BlockRight(),
                   #"center": pose.SitBlock(),
                   "center": pose.BlockCenter(),
                   }

        for name in blocks:
            b = blocks[name]
            self.add_transition(blocker, S(name), b, T(4.25), blocker)

        self.add_transition(blocker, S("penalty"), penalty)
        self.add_transition(penalty, S("blocker"), blocker)
