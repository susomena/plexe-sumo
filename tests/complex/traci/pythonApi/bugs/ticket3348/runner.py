#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file    runner.py
@author  Jakob Erdmann
@date    2017-01-23
@version $Id$


SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2017 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""

from __future__ import print_function
from __future__ import absolute_import
import os
import subprocess
import sys
import random
sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci
import sumolib  # noqa

sumoBinary = os.environ["SUMO_BINARY"]
PORT = sumolib.miscutils.getFreeSocketPort()
sumoProcess = subprocess.Popen([sumoBinary,
                                '-n', 'input_net.net.xml',
                                '-r', 'input_routes.rou.xml',
                                '--no-step-log',
                                #'-S', '-Q',
                                '--remote-port', str(PORT)], stdout=sys.stdout)


traci.init(PORT)
vehID = "v0"
stopPos = traci.lane.getLength("beg_0") * 0.9
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    step = traci.simulation.getCurrentTime() / 1000.0
    if step == 1:
        traci.vehicle.setStop(vehID, "beg", stopPos)
    elif step == 4:
        traci.vehicle.setStop(vehID, "beg", stopPos, duration=0)

traci.close()
sumoProcess.wait()
