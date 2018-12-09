"""NAOSAMI behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import math
import memory
import pose
import commands
import cfgstiff
from task import Task
import mem_objects
from state_machine import Node, C, T, LoopingStateMachine
from memory import localization_mem, game_state, robot_state
import core
import numpy as np
import os
import copy

beacon_data = []
a = [-1/2, -1/6, 0, 1/6, 1/2]
b = [-3*math.pi/4, -math.pi/2, -math.pi/4, 0,
      math.pi/4, math.pi/2, 3*math.pi/4, math.pi]
edge_thresh = 150
theta_thresh = 0.5
field_x = 1500
field_y = 1000
forward_x = np.arange(1, 6)
backward_x = np.array([0, 6, 7])

np.random.seed(42)

def get_vels(past_vels, vel_type=None):
    vel_choices = np.arange(len(b) - 1)
    if vel_type == "front":
        vel_choices = forward_x
    elif vel_type == "rear":
        vel_choices = backward_x

    if past_vels:
        vx, vy, vt = past_vels
        check = [vx, vy, vt]
    else:
        check = []

    while check == past_vels:
        a_ind = np.random.randint(0, len(a) - 1)
        np.random.shuffle(vel_choices)
        b_ind = vel_choices[0]

        rand_a = a[a_ind]
        rand_b = b[b_ind]

        vt = rand_a
        vx = math.cos(rand_b) * math.sqrt(1 - vt**2)
        vy = math.sin(rand_b) * math.sqrt(1 - vt**2)

        check = [vx, vy, vt]
        vel_choices = np.arange(len(b) - 1)

    return vx, vy, vt

class NewAction(Node):
    def __init__(self):
        super(NewAction, self).__init__()
        self.choose_new_action = True
        self.start_scan = -1
        self.frames_count = 0
        self.past_vels = []

    def run(self):
        state = game_state
        self.frames_count += 1

        self.start_scan = self.frames_count if self.start_scan == -1 else self.start_scan
        commands.setHeadTilt(-15)
        commands.setHeadPan(-math.pi / 5, 1.0)
        if self.frames_count - self.start_scan > 30: #0.5 sec at 100Hz (min 10)
            commands.setHeadPan(math.pi / 5, 1.0)
            if self.frames_count - self.start_scan > 60: #0.5 sec at 100Hz
                self.start_scan = -1

        if self.choose_new_action:
            # Front ~ ourKickOff
            if state.ourKickOff:
                print("Choosing forward velocity")
                vx, vy, vt = get_vels(self.past_vels, vel_type="front")
                state.ourKickOff = False
            # Rear ~ isFreeKick
            elif state.isFreeKick:
                print("Choosing backward velocity")
                vx, vy, vt = get_vels(self.past_vels, vel_type="rear")
                state.isFreeKick = False
            # Middle ~ isPenaltyKick
            elif state.isPenaltyKick:
                vx, vy, vt = get_vels(self.past_vels)
                state.isPenaltyKick = False
            # Time ran out
            else:
                vx, vy, vt = get_vels(self.past_vels)

            print(vx, vy, vt)

            commands.setWalkVelocity(vx, vy, vt)
            self.past_vels = [vx, vy, vt]
            self.choose_new_action = False

        vels = copy.deepcopy(self.past_vels)

        beacon1 = mem_objects.world_objects[core.WO_BEACON_BLUE_YELLOW]
        beacon2 = mem_objects.world_objects[core.WO_BEACON_YELLOW_BLUE]
        beacon3 = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK]
        beacon4 = mem_objects.world_objects[core.WO_BEACON_PINK_BLUE]
        beacon5 = mem_objects.world_objects[core.WO_BEACON_PINK_YELLOW]
        beacon6 = mem_objects.world_objects[core.WO_BEACON_YELLOW_PINK]

        beacons = [beacon1, beacon2, beacon3, beacon4, beacon5, beacon6]
        for i, beacon in enumerate(beacons):
            if beacon.seen:
#                     print('beacon', i, beacon.visionDistance, 'mm')
                vels.extend([beacon.beacon_height, beacon.visionBearing])
            else:
                vels.extend([None, None])

#             print(vels)
#             print("===\n")

        with open("beacon_data.txt", "a+") as f:
            np_data = np.array(vels, dtype=float)
            np.savetxt(f, np_data)
#                 f.write("====\n")

        if self.getTime() > 5.0:
            self.choose_new_action = True
            self.finish()

        if (state.isPenaltyKick or state.ourKickOff or state.isFreeKick):
            self.choose_new_action = True

class Playing(LoopingStateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                self.finish()

    def setup(self):
        stand = self.Stand()
        new_action = NewAction()

        # self.add_transition(stand, C, new_action)
        self.add_transition(new_action, C, new_action)

class Ready(Task):
    def run(self):
        commands.setStiffness()
        commands.stand()
        try:
            os.remove("beacon_data.txt")
        except OSError:
            pass
        if self.getTime() > 3.0:
            self.finish()

