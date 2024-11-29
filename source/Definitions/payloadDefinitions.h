/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_PAYLOADDEFINITIONS_H
#define GRS_SIMULATOR_PAYLOADDEFINITIONS_H

#pragma once

#include <vector>

struct payloadStates {
    double x;     // north-position-Earth   (m)
    double y;     // east-position-Earth    (m)
    double z;     // down-position-Earth    (m)
    double vx;    // north-velocity-Earth   (m/s)
    double vy;    // east-velocity-Earth    (m/s)
    double vz;    // down-velocity-Earth    (m/s)
    //double u;     // north-velocity-Body    (m/s)
    //double v;     // east-velocity-Body     (m/s)
    //double w;     // down-velocity-Body     (m/s)
    //double phi;   // roll angle             (rad)
    //double theta; // pitch angle            (rad)
    //double psi;   // yaw angle              (rad)
    //double p;     // roll rate              (rad/s)
    //double q;     // pitch rate             (rad/s)
    //double r;     // yaw rate               (rad/s)
};

struct payloadProperties {
    float mass{};
    std::vector<float> dimension{};
    float cda{};

//    double inertiaX{};  // kg*m^2
//    double inertiaY{};  // kg*m^2
//    double inertiaZ{};  // kg*m^2
//    double inertiaXY{}; // kg*m^2
//    double inertiaXZ{}; // kg*m^2
//    double inertiaYZ{}; // kg*m^2
};

#endif //GRS_SIMULATOR_PAYLOADDEFINITIONS_H
