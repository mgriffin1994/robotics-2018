"""Blank behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

from task import Task

import memory
import core

class Playing(Task):

    """Main behavior task."""

    def run(self):
        memory.speech.say('Hello, World!')
        print("============================= hi ==============================")
        for i in range(100):
            print("Iteration %d" % i)
            for i in range(core.NUM_SENSORS):
                print("sensor[%d]" % i, core.sensor_values[i])
            for i in range(core.NUM_JOINTS):
                print("joint[%d]" % i, core.joint_values[i])
        
        self.finish()
