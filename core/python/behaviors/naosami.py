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
from memory import localization_mem, game_state
import core
import numpy as np

beacon_data = []
a = [-1/2, -1/6, 0, 1/6, 1/2]
b = [-math.pi, -3*math.pi/4, -math.pi/2, -math.pi/4, 0, 
      math.pi/4, math.pi/2, 3*math.pi/4, math.pi]

np.random.seed(42)

def get_vels():
    a_ind = np.random.randint(0, len(a) - 1)
    b_ind = np.random.randint(0, len(b) - 1)
    rand_a = a[a_ind]
    rand_b = b[b_ind]

    vx = math.cos(rand_b)
    vy = math.sin(rand_b)
    vt = rand_a

    norm = vx**2 + vy**2 + vt**2
    vx /= norm
    vy /= norm
    vt /= norm

    return vx, vy, vt

class NewAction(Node):
    def __init__(self):
        super(NewAction, self).__init__()            
        self.done_walked = False
        self.start_scan = -1
        self.frames_count = 0
        self.past_vels = []

    def run(self):
        state = game_state
        self.frames_count += 1

        self.start_scan = self.frames_count if self.start_scan == -1 else self.start_scan
        commands.setHeadPan(-math.pi / 5, 0.40)
        if self.frames_count - self.start_scan > 20: #0.5 sec at 100Hz (min 10)
            commands.setHeadPan(math.pi / 5, 0.40)
            if self.frames_count - self.start_scan > 60: #0.5 sec at 100Hz
                self.start_scan = -1

        if not state.isPenaltyKick:
            vx, vy, vt = get_vels()

            if not self.done_walked:
                commands.setWalkVelocity(vx, vy, vt)
                self.past_vels = [vx, vy, vt]
                print(vx, vy, vt)
                self.done_walked = True

            vels = self.past_vels

            beacon1 = mem_objects.world_objects[core.WO_BEACON_BLUE_YELLOW]
            beacon2 = mem_objects.world_objects[core.WO_BEACON_YELLOW_BLUE]
            beacon3 = mem_objects.world_objects[core.WO_BEACON_BLUE_PINK]
            beacon4 = mem_objects.world_objects[core.WO_BEACON_PINK_BLUE]
            beacon5 = mem_objects.world_objects[core.WO_BEACON_PINK_YELLOW]
            beacon6 = mem_objects.world_objects[core.WO_BEACON_YELLOW_PINK]

            beacons = {beacon1, beacon2, beacon3, beacon4, beacon5, beacon6}
            for i, beacon in enumerate(beacons):
                if beacon.seen:
                    print('beacon', i, beacon.visionDistance, 'mm')
                    vels.extend([beacon.beacon_height, beacon.visionBearing])
                else:
                    vels.extend([None, None])

            beacon_data.append(vels)
            self.vels = []
            if self.getTime() > 5.0:
                self.done_walked = False
                self.finish()
        else:
            state.isPenaltyKick = False
            self.done_walked = False
            self.finish()

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

class Testing(Task):
    def __init__(self):
        super(Testing, self).__init__()            
        self.done_printed = False

    def run(self):
        if not self.done_printed:
            np_data = np.array(beacon_data, dtype=float)
            np.savetxt("beacon_data.txt", np_data)
            self.done_printed = True
