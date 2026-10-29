/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_FIXEDWINGDEFINITIONS_H
#define GRS_SIMULATOR_FIXEDWINGDEFINITIONS_H

#pragma once

#include "Mathematics/math.h"

/*
 *   FIXED-WING STRUCTURES
 */

struct fixedWingCommands {
    double altitude;
    double phi;
    double theta;
    double thrust;
};

struct fixedWingControlInputs {
    double deltaElevator;
    double deltaAileron;
    double deltaRudder;
    double deltaThrust;
};

struct fixedWingStates {
    double x;     // north-position-Earth   (m)
    double y;     // east-position-Earth    (m)
    double z;     // down-position-Earth    (m)
    double vx;    // north-velocity-Earth   (m/s)
    double vy;    // east-velocity-Earth    (m/s)
    double vz;    // down-velocity-Earth    (m/s)
    double u;     // north-velocity-Body    (m/s)
    double v;     // east-velocity-Body     (m/s)
    double w;     // down-velocity-Body     (m/s)
    double phi;   // roll angle             (rad)
    double theta; // pitch angle            (rad)
    double psi;   // yaw angle              (rad)
    double p;     // roll rate              (rad/s)
    double q;     // pitch rate             (rad/s)
    double r;     // yaw rate               (rad/s)
};

struct fixedWingProperties {
    // GEOMETRY
    double mass;           // kg
    double wingSurface;    // m^2
    double wingChord;      // m
    double wingSpan;       // m
    double aspectRatio;

    struct geometry {
        grs::Vec3d cgPosition{};
        grs::Vec3d cgOrientation{};
        grs::Vec3d aeroRefPosition{};
        grs::Vec3d aeroRefOrientation{};
        grs::Vec3d propulsionPosition{};
        grs::Vec3d propulsionOrientation{};
        grs::Vec3d tetherAttachPosition{};
        grs::Vec3d tetherAttachOrientation{};
    } geom;

    // INERTIA MATRIX
    double inertiaX;  // kg*m^2
    double inertiaY;  // kg*m^2
    double inertiaZ;  // kg*m^2
    double inertiaXY; // kg*m^2
    double inertiaXZ; // kg*m^2
    double inertiaYZ; // kg*m^2

    // LIMIT CONTROL SURFACES
    double ailMinDeg;  // deg
    double ailMaxDeg;  // deg
    double eleMinDeg;  // deg
    double eleMaxDeg;  // deg
    double rudMinDeg;  // deg
    double rudMaxDeg;  // deg

    // CALIBRATION TRIM (reference point of the aero model)
    double trimAlpha;       // deg
    double trimBeta;        // deg
    double trimElevator;    // deg
    double trimAileron;     // deg
    double trimRudder;      // deg

    // AERO COEFFICIENTS (calibrated model, same names as the MATLAB COEFF structure)
    // Angles and rates are in rad / rad/s, control deflections in deg.
    // Lift (stability axes)
    double CL_TRIM;
    double CL_ALPHA;            // 1/rad
    double CL_q;                // 1/rad (q * c / 2V)
    double CL_DEL_ELEVATOR;     // 1/deg
    // Drag (stability axes)
    double CD_0;
    double CD_ALPHA;            // 1/rad, on the absolute angle of attack
    double CD_BETA;             // 1/rad
    double CD_BETA2;            // 1/rad^2
    double CD_DEL_ELEVATOR;     // 1/deg, on |elevator|
    double CD_DEL_RUDDER;       // 1/deg, on |rudder|
    double e;                   // Oswald efficiency of the induced drag CL^2 / (pi * AR * e)
    // Side force
    double Cy_TRIM;
    double Cy_BETA;             // 1/rad
    double Cy_p;                // 1/rad (p * b / 2V)
    double Cy_r;                // 1/rad (r * b / 2V)
    double Cy_DEL_AILERON;      // 1/deg
    double Cy_DEL_RUDDER;       // 1/deg
    // Rolling moment
    double Cl_TRIM;
    double Cl_BETA;             // 1/rad
    double Cl_p;                // 1/rad
    double Cl_r;                // 1/rad
    double Cl_DEL_AILERON;      // 1/deg
    double Cl_DEL_RUDDER;       // 1/deg
    // Pitching moment
    double Cm_TRIM;
    double Cm_ALPHA;            // 1/rad
    double Cm_q;                // 1/rad (q * c / 2V)
    double Cm_DEL_ELEVATOR;     // 1/deg
    // Yawing moment
    double Cn_TRIM;
    double Cn_BETA;             // 1/rad
    double Cn_p;                // 1/rad
    double Cn_r;                // 1/rad
    double Cn_DEL_AILERON;      // 1/deg
    double Cn_DEL_RUDDER;       // 1/deg
};

#endif //GRS_SIMULATOR_FIXEDWINGDEFINITIONS_H
