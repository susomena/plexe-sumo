/****************************************************************************/
/// @file    CC_VehicleVariables.cpp
/// @author  Michele Segata
/// @date    Mon, 7 Mar 2016
/// @version $Id: $
///
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2011 DLR (http://www.dlr.de/) and contributors
// Copyright (C) 2012-2017 Michele Segata (segata@ccs-labs.org)
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#include "CC_VehicleVariables.h"

//initialize default L and K matrices
const int CC_VehicleVariables::defaultL[][MAX_N_CARS] =
    {
    {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 0},
    {1, 1, 0, 0, 0, 0, 0, 0},
    {1, 0, 1, 0, 0, 0, 0, 0},
    {1, 0, 0, 1, 0, 0, 0, 0},
    {1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 0, 0, 0, 1, 0, 0},
    {1, 0, 0, 0, 0, 0, 1, 0}
    };
const double CC_VehicleVariables::defaultK[][MAX_N_CARS] =
    {
    {0  , 0  , 0  , 0  , 0  , 0  , 0  , 0},
    {460, 0  , 0  , 0  , 0  , 0  , 0  , 0},
    {80 , 860, 0  , 0  , 0  , 0  , 0  , 0},
    {80 , 0  , 860, 0  , 0  , 0  , 0  , 0},
    {80 , 0  , 0  , 860, 0  , 0  , 0  , 0},
    {80 , 0  , 0  , 0  , 860, 0  , 0  , 0},
    {80 , 0  , 0  , 0  , 0  , 860, 0  , 0},
    {80 , 0  , 0  , 0  , 0  , 0  , 860, 0}
    };
const double CC_VehicleVariables::defaultB[MAX_N_CARS] = {1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800};
const double CC_VehicleVariables::defaultH[MAX_N_CARS] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};

CC_VehicleVariables::CC_VehicleVariables() :
    frontDataLastUpdate(0), frontSpeed(0), frontAcceleration(0),
    leaderDataLastUpdate(0), leaderSpeed(0), leaderAcceleration(0),
    platoonId(""), isPlatoonLeader(false), ccDesiredSpeed(14), activeController(Plexe::DRIVER),
    accHeadwayTime(1.5), useFixedAcceleration(0), fixedAcceleration(0),
    crashed(false), controllerAcceleration(0), caccSpacing(5),
    leaderDataReadTime(0), frontDataReadTime(0), position(-1), nCars(8),
    caccXi(-1), caccOmegaN(-1), caccC1(-1), engineTau(-1), caccAlpha1(-1), caccAlpha2(-1),
    caccAlpha3(-1), caccAlpha4(-1), caccAlpha5(-1), engineAlpha(-1), engineOneMinusAlpha(-1),
    ploegH(0.5), ploegKp(0.2), ploegKd(0.7), nInitialized(0), engine(0),
    frontInitialized(false), leaderInitialized(false), caccInitialized(false),
    engineModel(CC_ENGINE_MODEL_FOLM) {
    fakeData.frontAcceleration = 0;
    fakeData.frontDistance = 0;
    fakeData.frontSpeed = 0;
    fakeData.leaderAcceleration = 0;
    fakeData.leaderSpeed = 0;
    leaderPosition.set(0, 0);
    frontPosition.set(0, 0);
    //init L, K, b, and h with default values
    memcpy(L, defaultL, sizeof(int)*MAX_N_CARS*MAX_N_CARS);
    memcpy(K, defaultK, sizeof(double)*MAX_N_CARS*MAX_N_CARS);
    memcpy(b, defaultB, sizeof(double)*MAX_N_CARS);
    memcpy(h, defaultH, sizeof(double)*MAX_N_CARS);
    //no data about any vehicle has been set
    for (int i = 0; i < MAX_N_CARS; i++)
        initialized[i] = false;
}

CC_VehicleVariables::~CC_VehicleVariables() {
    if (engine)
        delete engine;
}
