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

def get_vels():
    a = [-1/2, -1/6, 0, 1/6, 1/2]
    b = [-math.pi, -3*math.pi/4, -math.pi/2, -math.pi/4, 0, 
          math.pi/4, math.pi/2, 3*math.pi/4, math.pi]

    rand_a = np.random.choice(a)
    rand_b = np.random.choice(b)

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
        self.start_scan = -1;

    def run(self):
        state = game_state

        self.start_scan = self.getFrames() if self.start_scan == -1 else self.start_scan
        commands.setHeadPan(-math.pi / 5, 0.10)
        if self.getFrames() - self.start_scan > 20: #0.5 sec at 100Hz (min 10)
            commands.setHeadPan(math.pi / 3, 0.4)
            if self.getFrames() - self.start_scan > 60: #0.5 sec at 100Hz
                commands.setHeadPan(0, 0.1)
                self.start_scan = -1

        if not state.isPenaltyKick:
            vx, vy, vt = get_vels()

            if not self.done_walked:
                commands.setWalkVelocity(vx, vy, vt)
                print(vx,vy,vt)
                self.done_walked = True

            vel = [vx, vy, vt]

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
                    vel.append((beacon.beacon_height, beacon.visionBearing))
                else:
                    vel.append((None, None))

            beacon_data.append(vel)
            if self.getTime() > 5.0:
                self.done_walked = False
                self.finish()
        else:
            state.isPenaltyKick = True
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

class Finished(Task):
    def run(self):
       np_data = np.array(beacon_data)
       np.save("beacon_data.txt", np_data)
