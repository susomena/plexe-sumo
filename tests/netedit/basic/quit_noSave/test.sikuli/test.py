#!/usr/bin/env python
"""
@file    test.py
@author  Pablo Alvarez Lopez
@date    2016-11-25
@version $Id: test.py 25910 2017-09-07 13:49:36Z namdre $

python script used by sikulix for testing netedit

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2009-2017 DLR/TS, Germany

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
# import common functions for netedit tests
import os
import sys

testRoot = os.path.join(os.environ.get('SUMO_HOME', '.'), 'tests')
neteditTestRoot = os.path.join(
    os.environ.get('TEXTTEST_HOME', testRoot), 'netedit')
sys.path.append(neteditTestRoot)
import neteditTestFunctions as netedit  # noqa

# Open netedit
neteditProcess, match = netedit.setupAndStart(neteditTestRoot)

# go to additional mode
netedit.additionalMode()

# select busStop
netedit.changeAdditional("busStop")

# create busStop in mode "reference left"
netedit.leftClick(match, 250, 250)

# go to shape mode
netedit.shapeMode()

# go to additional mode
netedit.changeShape("poly")

# create polygon
netedit.createSquaredPoly(match, 100, 50, 100, True)


# quit netedit without saving shapes
netedit.quit(neteditProcess, True, False, True, False, True, False)
