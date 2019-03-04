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

# go to shape mode
netedit.shapeMode()

# go to additional mode
netedit.changeShape("poly")

# create polygon
netedit.createSquaredPoly(match, 100, 150, 100, True)

# create polygon
netedit.createSquaredPoly(match, 200, 150, 100, True)

# enable block shape
netedit.modifyShapeDefaultBoolValue(11)

# create polygon blocked
netedit.createSquaredPoly(match, 300, 150, 100, True)

# disable block shape
netedit.modifyShapeDefaultBoolValue(11)

# enable block move
netedit.modifyShapeDefaultBoolValue(10)

# create polygon
netedit.createSquaredPoly(match, 400, 150, 100, True)

# enable block shape
netedit.modifyShapeDefaultBoolValue(11)

# create polygon
netedit.createSquaredPoly(match, 500, 150, 100, True)

# go to move mode
netedit.moveMode()

# move first polygon (only a existent vertex will be moved
netedit.moveElement(match, -90, 120, -90, 300)

# move second polygon (only a verte will be moved)
netedit.moveElement(match, 10, 120, 10, 300)

# move third polygon (entre shape will be moved)
netedit.moveElement(match, 130, 120, 130, 300)

# move four polygon (will not be moved
netedit.moveElement(match, 260, 120, 260, 300)

# move five polygon (will not be moved
netedit.moveElement(match, 380, 120, 380, 300)

# Check undo redo
netedit.undo(match, 8)
netedit.redo(match, 8)

# save shapes
netedit.saveShapes()

# save newtork
netedit.saveNetwork()

# quit netedit
netedit.quit(neteditProcess)

