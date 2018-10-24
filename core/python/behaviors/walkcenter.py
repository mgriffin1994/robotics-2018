"""Simple walk center behavior."""

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
from memory import robot_state
import numpy as np

x_kp = 5e-4
x_ki = 0.0
x_kd = 0.0
y_kp = 0.0
y_ki = 0.0
y_kd = 0.0
theta_kp = 2e-1
theta_kd = 0.0
theta_ki = 0.0


# x_kp = 0.0
# x_ki = 0.0
# x_kd = 0.0
# y_kp = 0.0
# y_ki = 0.0
# y_kd = 0.0
# theta_kp = 0.0
# theta_kd = 0.0
# theta_ki = 0.0

time_delay = 7.5
eps = 0.1


max_length = 10
max_int = 10
moving_avg_samples = 3
max_num_frames = 30
max_int_steps = 10 #if want to use all steps, then set to float("inf")

center_distance_close = 50

x_error_thresh = 100 #w/in 12 cm of center_distance_close around center
y_error_thresh = 0.1
theta_error_thresh = math.pi / 8

vel_thresh = 0.1

top_cam_width = 320
bot_cam_width = 320


class ApproachCenter(Node):

    def __init__(self):
        super(ApproachCenter, self).__init__()
        self.x_errors = [x_error_thresh] * max_int #assume start out with at least x_error_thresh (and not below)
        self.y_errors = [y_error_thresh] * max_int
        self.theta_errors = [theta_error_thresh] * max_int
        self.prev_time = 1e-6
        self.prev_center_distance = x_error_thresh + center_distance_close
        self.prev_goal_centerx = 0
        self.prev_center_centerx = 0
        self.goal_search_done = False
        self.center_search_done = False
        self.start_driving = False
        self.avg_time_step = 0.01

        self.num_frames_not_seen_beacon = max_num_frames
        self.num_beacons_seen = 0
        self.first_beacon = -1

        self.frames = 0
        self.start_time = -1

    def run(self):

        commands.setHeadTilt(5)


        # if self.frames < 300:
        #     commands.setWalkVelocity(0.3, 0, 0) #less than 0.2 goes backwards kinda
        # elif self.frames <= 600:
        #     commands.setWalkVelocity(0, 0, 0) #less than 0.2 goes backwards kinda
        # else:
        #     commands.setWalkVelocity(-0.3, 0, 0) #less than 0.2 goes backwards kinda
        # if self.frames > 900:
        #     self.frames = 0
        # self.frames += 1
        # return



        # pan = core.joint_values[core.HeadPan]
        # print('pan',pan)
        # if abs(pan - math.pi / 3) < eps: #if within eps from full right, turn left
        #     commands.setHeadPan(-math.pi/3, time_delay)
        # elif abs(pan + math.pi / 3) < eps: #if within eps from full left, turn right
        #     commands.setHeadPan(math.pi/3, time_delay)

        beacons = []
        beacons.append(mem_objects.world_objects[core.WO_BEACON_BLUE_YELLOW])
        beacons.append(mem_objects.world_objects[core.WO_BEACON_YELLOW_BLUE])
        beacons.append(mem_objects.world_objects[core.WO_BEACON_BLUE_PINK])
        beacons.append(mem_objects.world_objects[core.WO_BEACON_PINK_BLUE])
        beacons.append(mem_objects.world_objects[core.WO_BEACON_PINK_YELLOW])
        beacons.append(mem_objects.world_objects[core.WO_BEACON_YELLOW_PINK])

        beacons_seen = [beacon.seen for beacon in beacons]

        rob = mem_objects.world_objects[robot_state.WO_SELF]

        rob_x = rob.loc.x
        rob_y = rob.loc.y
        rob_t = rob.orientation
        
        print('rob_x', rob_x, 'rob_y', rob_y, 'rob_t', rob_t)

        if self.num_frames_not_seen_beacon < max_num_frames:

             ###Compute errors
            x_error = math.sqrt(rob_x**2 + rob_y**2)
            x_error_avg = (x_error + sum(self.x_errors[-moving_avg_samples+1:])) / moving_avg_samples

            
            y_error = 0.0
            y_error_avg = (y_error + sum(self.y_errors[-moving_avg_samples+1:])) / moving_avg_samples


            theta_error = math.atan2(-rob_y, -rob_x) - rob_t
            theta_error_avg = (theta_error + sum(self.theta_errors[-moving_avg_samples+1:])) / moving_avg_samples
            print('x_error', x_error, 'theta_error', theta_error)


            ###Add to previous errors
            self.x_errors.append(x_error)
            self.y_errors.append(y_error)
            self.theta_errors.append(theta_error)
            if len(self.x_errors) > max_int_steps:
                self.x_errors.pop(0)
            if len(self.y_errors) > max_int_steps:
                self.y_errors.pop(0)
            if len(self.theta_errors) > max_int_steps:
                self.theta_errors.pop(0)


            time = self.getTime()

            ##Compute velocities
            prev_x_avg = sum(self.x_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            x_vel = x_kp*x_error_avg + (x_kd*(x_error_avg - prev_x_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) + x_ki*(max(x_error + sum(self.x_errors), max_int))

            if (x_vel < 0.05):
                x_vel = 0.0


            prev_y_avg = sum(self.y_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            y_vel = y_kp*y_error_avg + (y_kd*(y_error_avg - prev_y_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) + y_ki*(max(y_error + sum(self.y_errors), max_int))


            prev_theta_avg = sum(self.theta_errors[-1-moving_avg_samples:-1]) / moving_avg_samples
            theta_vel = theta_kp*theta_error_avg + (theta_kd*(theta_error_avg - prev_theta_avg) / (2*(max(time - self.prev_time + 1e-5, self.avg_time_step)))) + theta_ki*(max(theta_error + sum(self.theta_errors), max_int))

            print('x_vel', x_vel, 'theta_vel', theta_vel)

            

            if (abs(x_error_avg) < x_error_thresh or self.start_time != -1):
                print("Within threshold")
                commands.stand()
                self.start_time = self.getTime()
                if self.getTime() - self.start_time > 10:
                    self.start_time = -1
            else:
                commands.setWalkVelocity(x_vel, 0, theta_vel)
                #commands.setWalkVelocity(0.3, 0, 0) #less than 0.2 goes backwards kinda
                #pass

            self.prev_time = time
        
        else:
            
            beacon_count = sum(1 for i, beacon in enumerate(beacons_seen) if beacon and (i != self.first_beacon or self.first_beacon == -1))
            self.num_beacons_seen += beacon_count

            if self.num_beacons_seen == 1:
                self.first_beacon = next(i for i in enumerate(beacons_seen))

            if self.num_beacons_seen >= 2:
                self.num_beacons_seen = 0
                self.num_frames_not_seen_beacon = 0
                self.first_beacon = -1
            else:
                #commands.setWalkVelocity(0, 0, math.pi/3)
                pass

        if not any(beacons_seen):
            self.num_frames_not_seen_beacon += 1

class Ready(Task):
    def run(self):
        commands.setStiffness()
        #commands.setHeadPan(math.pi/3, 1.0)
        commands.stand()
        commands.setHeadTilt(5)

        if self.getTime() > 3.0:
            self.finish()

class Playing(LoopingStateMachine):
    def setup(self):
        center_approach = ApproachCenter()

        self.add_transition(center_approach)
