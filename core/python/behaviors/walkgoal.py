"""Simple goal follow behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import core
import commands
import mem_objects
from state_machine import Node, S, T, C, LoopingStateMachine
import UTdebug

goal_threshold_min = 1900
goal_threshold_max = 3000

class WalkTowardsGoal(Node):
    def run(self):
        goal = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]
        commands.setStiffness()
        print(goal.visionDistance, goal.seen)
        if goal.seen:
            if goal.visionDistance > goal_threshold_max:
                commands.setWalkVelocity(0.5, 0, goal.visionBearing)
            elif goal.visionDistance < goal_threshold_min:
                commands.setWalkVelocity(-0.5, 0, goal.visionBearing)
            elif goal.visionDistance >= goal_threshold_min \
                and goal.visionDistance <= goal_threshold_max:
                commands.setWalkVelocity(0, 0, 0)
                # print(goal.visionBearing, goal.visionElevation)
                # print(goal.seen, goal.imageCenterX, goal.imageCenterY)

class Playing(LoopingStateMachine):
    def setup(self):
        goal_walk = WalkTowardsGoal()
        self.add_transition(goal_walk)
