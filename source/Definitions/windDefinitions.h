/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_WINDDEFINITIONS_H
#define GRS_SIMULATOR_WINDDEFINITIONS_H

#pragma once
#include <cstdint>

enum class Profile { None, Ramp, PowerLaw };

struct windProperties {
    bool wind;
    bool turbulence;
    uint64_t seed;                  // For reproducibility of turbulence
    std::vector<double> meanWind;   // [m/s] [wx wy wz]
    std::string meanWindPofile;     // NONE / RAMP / POWERLAW
    double meanWindGroundHeight;    // No wind below [m]
    double meanWindFullHeight;      // Full wind above [m]
    std::vector<double> sigma;      // Turbulence intensity [m/s] [sig_wx sig_wy sig_wz]
    double turbulenceGroundHeight;  // No turbulence below [m]
    double turbulenceFullHeight;    // Full turbulence above [m]
    double turbulenceLengthScale;   // Integral length scale [m]
    double turbulenceLambdaMin;     // Minimum wavelength [m]
    double turbulenceLambdaMax;     // Maximum wavelength [m]
    int turbulenceNumberMode;       // Total number of Fourier modes (must be even)
};


#endif //GRS_SIMULATOR_WINDDEFINITIONS_H
