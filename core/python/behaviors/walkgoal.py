"""Simple goal follow behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
from state_machine import Node, S, T, C, LoopingStateMachine
import UTdebug



class WalkTowardsGoal(Node):
    def run(self):
        goal = mem_objects.world_objects[core.WO_OWN_GOAL]
        print(goal.distance, goal.seen)
        if goal.seen and goal.distance > 100:
            commands.setWalkVelocity(0.5, 0, goal.visionBearing)
        elif goal.seen and goal.distance <= 100:
            commands.setWalkVelocity(0, 0, 0)
            print(goal.visionBearing, goal.visionElevation)
            print(goal.seen, goal.imageCenterX, goal.imageCenterY)

class Playing(LoopingStateMachine):
    def setup(self):
        goal_walk = WalkTowardsGoal()
        self.add_transition(goal_walk)
