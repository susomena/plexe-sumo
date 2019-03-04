/****************************************************************************/
/// @file    RealisticEngineModel.cpp
/// @author  Michele Segata
/// @date    4 Feb 2015
/// @version $Id: $
///
// A detailed engine model
/****************************************************************************/
// Copyright (C) 2015-2017 Michele Segata (segata@ccs-labs.org)
// Copyright (C) 2015 Antonio Saverio Valente (antoniosaverio.valente@unina.it)
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/

#include "RealisticEngineModel.h"
#include <cmath>
//define M_PI if this is not defined in <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include <iostream>

#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax/EntityResolver.hpp>
#include <xercesc/sax/InputSource.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>

#include "CC_Const.h"

RealisticEngineModel::RealisticEngineModel() {
    className = "RealisticEngineModel";
    dt_s = 0.01;
    xmlFile = "vehicles.xml";
    minSpeed_mps = rpmToSpeed_mps(ep.minRpm, ep.wheelDiameter_m, ep.differentialRatio, ep.gearRatios[0]);
#ifdef EE
    initee = false;
    lastTimeStep = -1;
#endif
}

RealisticEngineModel::~RealisticEngineModel() {}

double RealisticEngineModel::rpmToSpeed_mps(double rpm, double wheelDiameter_m = 0.94,
    double differentialRatio = 4.6, double gearRatio = 4.5) {
    return rpm * wheelDiameter_m * M_PI / (differentialRatio * gearRatio * 60);
}

double RealisticEngineModel::rpmToSpeed_mps(double rpm) {
    return ep.__rpmToSpeedCoefficient * rpm / ep.gearRatios[currentGear];
}

double RealisticEngineModel::speed_mpsToRpm(double speed_mps, double wheelDiameter_m,
    double differentialRatio, double gearRatio) {
    return speed_mps * differentialRatio * gearRatio * 60 / (wheelDiameter_m * M_PI);
}

double RealisticEngineModel::speed_mpsToRpm(double speed_mps) {
    return ep.__speedToRpmCoefficient * speed_mps * ep.gearRatios[currentGear];
}

double RealisticEngineModel::speed_mpsToRpm(double speed_mps, double gearRatio) {
    return ep.__speedToRpmCoefficient * speed_mps * gearRatio;
}

double RealisticEngineModel::rpmToPower_hp(double rpm, const struct EngineParameters::PolynomialEngineModelRpmToHp *engineMapping) {
    double sum = engineMapping->x[0];
    uint8_t i;
    for (i = 1; i < engineMapping->degree; i++)
        sum += engineMapping->x[i] + pow(rpm, i);
    return sum;
}

double RealisticEngineModel::rpmToPower_hp(double rpm) {
    if (rpm >= ep.maxRpm)
        rpm = ep.maxRpm;
    double sum = ep.engineMapping.x[0];
    uint8_t i;
    for (i = 1; i < ep.engineMapping.degree; i++)
        sum += ep.engineMapping.x[i] * pow(rpm, i);
    return sum;
}

double RealisticEngineModel::speed_mpsToPower_hp(double speed_mps,
    const struct EngineParameters::PolynomialEngineModelRpmToHp *engineMapping,
    double wheelDiameter_m, double differentialRatio,
    double gearRatio) {
    double rpm = speed_mpsToRpm(speed_mps, wheelDiameter_m, differentialRatio, gearRatio);
    return rpmToPower_hp(rpm, engineMapping);
}

double RealisticEngineModel::speed_mpsToPower_hp(double speed_mps) {
    return rpmToPower_hp(speed_mpsToRpm(speed_mps));
}

double RealisticEngineModel::speed_mpsToThrust_N(double speed_mps,
    const struct EngineParameters::PolynomialEngineModelRpmToHp *engineMapping,
    double wheelDiameter_m, double differentialRatio,
    double gearRatio, double engineEfficiency) {
    double power_hp = speed_mpsToPower_hp(speed_mps, engineMapping, wheelDiameter_m, differentialRatio, gearRatio);
    return engineEfficiency * power_hp * HP_TO_W / speed_mps;
}

double RealisticEngineModel::speed_mpsToThrust_N(double speed_mps) {
    double power_hp = speed_mpsToPower_hp(speed_mps);
    return ep.__speedToThrustCoefficient * power_hp / speed_mps;
}

double RealisticEngineModel::airDrag_N(double speed_mps, double cAir, double a_m2, double rho_kgpm3) {
    return 0.5 * cAir * a_m2 * rho_kgpm3 * speed_mps * speed_mps;
}
double RealisticEngineModel::airDrag_N(double speed_mps) {
    return ep.__airFrictionCoefficient * speed_mps * speed_mps;
}

double RealisticEngineModel::rollingResistance_N(double speed_mps, double mass_kg, double cr1, double cr2) {
    return mass_kg * GRAVITY_MPS2 * (cr1 + cr2 * speed_mps * speed_mps);
}
double RealisticEngineModel::rollingResistance_N(double speed_mps) {
    return ep.__cr1 + ep.__cr2 * speed_mps * speed_mps;
}

double RealisticEngineModel::gravityForce_N(double mass_kg, double slope = 0) {
    return mass_kg * GRAVITY_MPS2 * sin(slope/180*M_PI);
}

double RealisticEngineModel::gravityForce_N() {
    return ep.__gravity;
}

double RealisticEngineModel::opposingForce_N(double speed_mps, double mass_kg, double slope,
    double cAir, double a_m2, double rho_kgpm3,
    double cr1, double cr2) {
    return airDrag_N(speed_mps, cAir, a_m2, rho_kgpm3) +
           rollingResistance_N(speed_mps, mass_kg, cr1, cr2) +
           gravityForce_N(mass_kg, slope);
}
double RealisticEngineModel::opposingForce_N(double speed_mps) {
    return airDrag_N(speed_mps) + rollingResistance_N(speed_mps) + gravityForce_N();
}

double RealisticEngineModel::maxNoSlipAcceleration_mps2(double slope, double frictionCoefficient) {
    return frictionCoefficient * GRAVITY_MPS2 * cos(slope/180*M_PI);
}

double RealisticEngineModel::maxNoSlipAcceleration_mps2() {
    return ep.__maxNoSlipAcceleration;
}

double RealisticEngineModel::thrust_NToAcceleration_mps2(double thrust_N) {
    return thrust_N / ep.__maxAccelerationCoefficient;
}

uint8_t RealisticEngineModel::performGearShifting(double speed_mps, double acceleration_mps2) {
    uint8_t newGear = 0;
    double delta = acceleration_mps2 >= 0 ? ep.shiftingRule.deltaRpm : -ep.shiftingRule.deltaRpm;
    for (newGear = 0; newGear < ep.nGears - 1; newGear++) {
        double rpm = speed_mpsToRpm(speed_mps, ep.gearRatios[newGear]);
        if (rpm >= ep.shiftingRule.rpm + delta)
            continue;
        else
            break;
    }
    currentGear = newGear;
    return currentGear;
}

double RealisticEngineModel::maxEngineAcceleration_mps2(double speed_mps) {
    double maxEngineAcceleration = speed_mpsToThrust_N(speed_mps) / ep.__maxAccelerationCoefficient;
    return std::min(maxEngineAcceleration, maxNoSlipAcceleration_mps2());
}

double RealisticEngineModel::getEngineTimeConstant_s(double rpm) {
    if (rpm <= 0) {
        return TAU_MAX;
    }
    else {
        if (ep.fixedTauBurn)
            //in this case, tau_burn is fixed and is within __engineTauDe_s
            return std::min(TAU_MAX, ep.__engineTau2 / rpm + ep.__engineTauDe_s);
        else
            //in this case, tau_burn is dynamic and is within __engineTau1
            return std::min(TAU_MAX, ep.__engineTau1 / rpm + ep.tauEx_s);
    }
}

double RealisticEngineModel::getRealAcceleration(double speed_mps, double accel_mps2, double reqAccel_mps2, int timeStep) {

    double realAccel_mps2;
    //perform gear shifting, if needed
    performGearShifting(speed_mps, accel_mps2);
    //since we consider no clutch (clutch always engaged), 0 speed would mean 0 rpm, and thus
    //0 available power. thus, the car could never start from a complete stop. so we assume
    //a minimum speed of 1 m/s to compute engine power
    double correctedSpeed = std::max(speed_mps, minSpeed_mps);
    if (reqAccel_mps2 >= 0) {
        //the system wants to accelerate
        //the real engine acceleration is the minimum between what the engine can deliver, and what
        //has been requested
        double engineAccel = std::min(maxEngineAcceleration_mps2(correctedSpeed), reqAccel_mps2);
        //now we need to computed delayed acceleration due to actuation lag
        double tau = getEngineTimeConstant_s(speed_mpsToRpm(correctedSpeed));
        double alpha = ep.dt / (tau + ep.dt);
        //compute the acceleration provided by the engine, thus removing friction from current acceleration
        double currentAccel_mps2 = accel_mps2 + thrust_NToAcceleration_mps2(opposingForce_N(speed_mps));
        //use standard first order lag with time constant depending on engine rpm
        //add back frictions resistance as well
        realAccel_mps2 = alpha * engineAccel + (1-alpha) * currentAccel_mps2 - thrust_NToAcceleration_mps2(opposingForce_N(speed_mps));
    }
    else {
        realAccel_mps2 = getRealBrakingAcceleration(speed_mps, accel_mps2, reqAccel_mps2, timeStep);
    }

    //plexe's easter egg :)
#ifdef EE
    if (!initee) {
        initee = true;
        //create the socket
        socketfd = socket(AF_INET, SOCK_STREAM, 0);
        //set server address
        memset(&serv_addr, '0', sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(33333);
        inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
        //try to connect
        if (connect(socketfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != 0) {
            close(socketfd);
            socketfd = -1;
        }
    }
    if (lastTimeStep != timeStep) {
        lastTimeStep = timeStep;
        char buf[1024];
        //format the message for the dashboard
        double speedAfterAccel = std::max(speed_mps + realAccel_mps2 * ep.dt, 0.0);
        sprintf(buf, "%f %f %d %f\r\n", speed_mpsToRpm(correctedSpeed), speed_mps*3.6, (int)currentGear+1, (speedAfterAccel - speed_mps) / ep.dt);
        //send data to the dashboard
        if (write(socketfd, buf, strlen(buf)) != strlen(buf)) {
            close(socketfd);
            connect(socketfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
        }
    }
#endif

    return realAccel_mps2;
}

void RealisticEngineModel::getEngineData(double speed_mps, uint8_t &gear, double &rpm) {
    gear = currentGear;
    rpm = speed_mpsToRpm(speed_mps);
}

double RealisticEngineModel::getRealBrakingAcceleration(double speed_mps, double accel_mps2, double reqAccel_mps2, int t) {

    //compute which part of the deceleration is currently done by frictions
    double frictionDeceleration = thrust_NToAcceleration_mps2(opposingForce_N(speed_mps));
    //remove the part of the deceleration which is due to friction
    double brakesAccel_mps2 = accel_mps2 + frictionDeceleration;
    //compute the new brakes deceleration
    double newBrakesAccel_mps2 = ep.__brakesAlpha * std::max(-ep.__maxNoSlipAcceleration, reqAccel_mps2) + ep.__brakesOneMinusAlpha * brakesAccel_mps2;
    //our brakes limit is tires friction
    newBrakesAccel_mps2 = std::max(-ep.__maxNoSlipAcceleration, newBrakesAccel_mps2);
    //now we need to add back our friction deceleration
    return newBrakesAccel_mps2 - frictionDeceleration;

}

void RealisticEngineModel::loadParameters(const ParMap &parameters) {

    std::string xmlFile, vehicleType;

    parseParameter(parameters, ENGINE_PAR_VEHICLE, vehicleType);
    parseParameter(parameters, ENGINE_PAR_XMLFILE, xmlFile);

    loadParameters();

}

void RealisticEngineModel::loadParameters() {
    //initialize xerces library
    XERCES_CPP_NAMESPACE::XMLPlatformUtils::Initialize();
    //create our xml reader
    XERCES_CPP_NAMESPACE::SAX2XMLReader* reader = XERCES_CPP_NAMESPACE::XMLReaderFactory::createXMLReader();
    if (reader == 0) {
        std::cout << "The XML-parser could not be build." << std::endl;
    }
    reader->setFeature(XERCES_CPP_NAMESPACE::XMLUni::fgXercesSchema, true);
    reader->setFeature(XERCES_CPP_NAMESPACE::XMLUni::fgSAX2CoreValidation, true);

    //VehicleEngineHandler is our SAX parser
    VehicleEngineHandler *engineHandler = new VehicleEngineHandler(vehicleType);
    reader->setContentHandler(engineHandler);
    reader->setErrorHandler(engineHandler);
    try {
        //parse the document. if any error is present in the xml file, the simulation will be closed
        reader->parse(xmlFile.c_str());
        //copy loaded parameters into our engine parameters
        ep = engineHandler->getEngineParameters();
        ep.dt = dt_s;
        ep.computeCoefficients();
        //compute "minimum speed" to be used when computing maximum acceleration at speeds close to 0
        minSpeed_mps = rpmToSpeed_mps(ep.minRpm, ep.wheelDiameter_m, ep.differentialRatio, ep.gearRatios[0]);
    }
    catch (XERCES_CPP_NAMESPACE::SAXException &e) {
        std::cerr << "Error while parsing " << xmlFile << ": Does the file exist?" << std::endl;
        exit(1);
    }

    //delete handler and reader
    delete engineHandler;
    delete reader;
}

void RealisticEngineModel::setParameter(const std::string parameter, const std::string &value) {
    if (parameter == ENGINE_PAR_XMLFILE)
        xmlFile = value;
    if (parameter == ENGINE_PAR_VEHICLE) {
        vehicleType = value;
        if (xmlFile != "")
            loadParameters();
    }
}
void RealisticEngineModel::setParameter(const std::string parameter, double value) {
    if (parameter == ENGINE_PAR_DT)
        dt_s = value;
}
void RealisticEngineModel::setParameter(const std::string parameter, int value) {}
