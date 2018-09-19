"""Sample behavior."""

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
from state_machine import Node, C, T, StateMachine



class Playing(StateMachine):
    class Stand(Node):
        def run(self):
            commands.stand()
            if self.getTime() > 5.0:
                self.finish()

    class TurnBodyLeftInPlace(Node):
        def run(self):
            commands.setWalkVelocity(0, 0, 0.1)

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




    def setup(self):
        stand = self.Stand()
        body_left_in_place = self.TurnBodyLeftInPlace()

        self.trans(stand, C,
                   body_left_in_place)
