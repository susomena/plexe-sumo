/****************************************************************************/
/// @file    CC_VehicleVariables.h
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
#ifndef CC_VEHICLEVARIABLES_H
#define CC_VEHICLEVARIABLES_H

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "CC_Const.h"
#include <microsim/cfmodels/MSCFModel.h>
#include <utils/geom/Position.h>
#include <string.h>

#include "GenericEngineModel.h"
#include "FirstOrderLagModel.h"
#include "RealisticEngineModel.h"

class CC_VehicleVariables : public MSCFModel::VehicleVariables {
public:

    /**
     * @struct FAKE_CONTROLLER_DATA
     * @brief represent the set of fake data which is sent to the controller in
     * order to automatically make the car move to a precise position before
     * joining the platoon.
     * we expect to get from the upper application the data that the CACC needs, i.e.:
     * - front distance, front speed and front vehicle acceleration: this information
     *   regards the car that the vehicle joining the platoon will have directly in
     *   front. this data might be real or might be fake: for example, if the platoon
     *   management algorithm decides to set the vehicle as the new leader, there won't
     *   be a car in front, and the fake data will be used only for positioning. in the
     *   case of fake data, acceleration must be set to 0
     * - leader front speed and acceleration: this information is the same as previously
     *   described for vehicle in front, but regards the leader. again, if the vehicle
     *   is being set as the new leader, this data might be fake data
     */
    struct FAKE_CONTROLLER_DATA {
        double frontDistance;
        double frontSpeed;
        double frontAcceleration;
        double leaderSpeed;
        double leaderAcceleration;
    };

    /**
     * Topology matrix L for the consensus controller
     */
    const static int defaultL[MAX_N_CARS][MAX_N_CARS];

    /**
     * Gains matrix K for the consensus controller
     */
    const static double defaultK[MAX_N_CARS][MAX_N_CARS];

    /**
     * Default damping ratios vector b for the consensus controller
     */
    const static double defaultB[];

    /**
     * Default time headways vector h for the consensus controller
     */
    const static double defaultH[];

    CC_VehicleVariables();
    ~CC_VehicleVariables();

    /// @brief acceleration as computed by the controller, to be sent to other vehicles
    double controllerAcceleration;

    /// @brief last time front vehicle data (speed and acceleration) has been updated
    SUMOTime frontDataLastUpdate;
    /// @brief current front vehicle speed
    double frontSpeed;
    /// @brief current front vehicle acceleration (used by CACC)
    double frontAcceleration;
    /// @brief current front vehicle position
    Position frontPosition;
    /// @brief when front vehicle data has been readed from GPS
    double frontDataReadTime;
    /// @did we receive at least one packet?
    bool frontInitialized;

    /// @brief headway time for ACC
    double accHeadwayTime;
    /// @brief fixed spacing for CACC
    double caccSpacing;

    /// @brief last time leader vehicle data has been updated
    SUMOTime leaderDataLastUpdate;
    /// @brief platoon's leader speed (used by CACC)
    double leaderSpeed;
    /// @brief platoon's leader acceleration (used by CACC)
    double leaderAcceleration;
    /// @brief platoon's leader position
    Position leaderPosition;
    /// @brief when leader data has been readed from GPS
    double leaderDataReadTime;
    /// @did we receive at least one packet?
    bool leaderInitialized;
    bool caccInitialized;

    //enable/disable the use of a constant, user defined acceleration instead of the one computed by the controller
    int useFixedAcceleration;
    //fixed acceleration to use
    double fixedAcceleration;

    //car collided in the last timestep
    bool crashed;

    /// @brief CC desired speed
    double ccDesiredSpeed;
    /// @brief currently active controller
    enum Plexe::ACTIVE_CONTROLLER activeController;

    /// @brief fake controller data. @see FAKE_CONTROLLER_DATA
    struct FAKE_CONTROLLER_DATA fakeData;

    //TODO: most probably the following variables needs to be moved to the application logic (i.e., network protocol)
    /// @brief own platoon id
    std::string platoonId;
    /// @brief is ego vehicle the leader?
    bool isPlatoonLeader;

    /// @brief L matrix
    int L[MAX_N_CARS][MAX_N_CARS];
    /// @brief K matrix
    double K[MAX_N_CARS][MAX_N_CARS];
    /// @brief vector of damping ratios b
    double b[MAX_N_CARS];
    /// @brief vector of time headways h
    double h[MAX_N_CARS];

    /// @brief data about vehicles in the platoon
    struct Plexe::VEHICLE_DATA vehicles[MAX_N_CARS];
    /// @brief tells whether data about a certain vehicle has been initialized
    bool initialized[MAX_N_CARS];
    /// @brief count of initialized vehicles
    int nInitialized;
    /// @brief my position within the platoon (0 = first car)
    int position;
    /// @brief number of cars in the platoon
    int nCars;

    /// @brief controller related parameters
    double caccXi;
    double caccOmegaN;
    double caccC1;
    double caccAlpha1, caccAlpha2, caccAlpha3, caccAlpha4, caccAlpha5;
    double engineTau, engineAlpha, engineOneMinusAlpha;
    double ploegH;
    double ploegKp;
    double ploegKd;

    /// @brief engine model employed by this car
    GenericEngineModel *engine;
    /// @brief numeric value indicating the employed model
    int engineModel;
};

#endif
