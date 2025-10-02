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
    double oswaldEfficiency;

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

    // Model Baseline (0)
    double trimElevator;   // deg
    double trimAlpha;      // deg

    // AERO COEFFICIENTS
    // Lift and Drag
    double CL_0;            // rad
    double CL_ALPHA;        // rad
    double CD_0;            // rad (CD_vis in avl)
    // Force x direction
    double Cx_q;            // rad
    double Cx_DELTA_ELE;    // deg
    // Force y direction
    double Cy_0;            // rad
    double Cy_BETA;         // rad
    double Cy_p;            // rad
    double Cy_r;            // rad
    double Cy_DELTA_AIL;    // deg
    double Cy_DELTA_RUD;    // deg
    // Force z direction
    double Cz_q;            // rad
    double Cz_DELTA_ELE;    // deg
    // Moment l
    double Cl_0;            // rad
    double Cl_BETA;         // rad
    double Cl_p;            // rad
    double Cl_r;            // rad
    double Cl_DELTA_AIL;    // deg
    double Cl_DELTA_RUD;    // deg
    //Moment m
    double Cm_0;            // rad
    double Cm_ALPHA;        // rad
    double Cm_q;            // rad
    double Cm_DELTA_ELE;    // deg
    // Moment n
    double Cn_0;            // rad
    double Cn_BETA;         // rad
    double Cn_p;            // rad
    double Cn_r;            // rad
    double Cn_DELTA_AIL;    // deg
    double Cn_DELTA_RUD;    // deg
};

#endif //GRS_SIMULATOR_FIXEDWINGDEFINITIONS_H
