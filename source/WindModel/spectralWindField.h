/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_SPECTRALWINDFIELD_H
#define GRS_SIMULATOR_SPECTRALWINDFIELD_H

#include <vector>
#include <random>

#include "turbulenceModel.h"

/*
 * Pure isotropic homogeneous turbulence generator, based on the
 * von Karman spectrum synthesized via the Random Fourier Mode /
 * Kinematic Simulation method (Kraichnan 1970; Fung, Hunt, Malik &
 * Perkins 1992; Shinozuka & Deodatis 1996).
 *
 * DELIBERATELY SCOPED: this class knows nothing about mean wind
 * profiles, altitude ramps, launch phases, or any mission-specific
 * modeling. It answers exactly one question -- "what is the
 * turbulent velocity fluctuation at this position and time, given
 * this advection velocity?" -- and nothing else.
 */
class SpectralWindField final : public TurbulenceModel {

public:

    struct params {
        grs::Vec3d sigma{0.5, 0.4, 0.25};        // [u v w] RMS turbulence velocity [m/s]
        double L = 150.0;                        // integral length scale [m]
        double lambdaMin = 0.5;                  // minimum wavelength [m]
        double lambdaMax = 2000.0;               // maximum wavelength [m]
        int numberModes = 1024;                  // total number of Fourier modes (must be even)
        uint64_t seed = 1;                       // random seed
    };

    explicit SpectralWindField(const params& parameters);
    [[nodiscard]] grs::Vec3d getTurbulence(const grs::Vec3d& position, const grs::Vec3d& advectionVelocity, double time) const override;

    [[nodiscard]] const grs::Vec3d& scaleFactor() const { return m_scaleFactor; };
    [[nodiscard]] const grs::Vec3d& uncalibratedRMS() const { return m_rmsUncalibrated; };
    [[nodiscard]] const params& parameters() const { return m_parameters; }

private:

    // One Fourier mode
    struct fourierMode {
        grs::Vec3d k;           // Wave vector
        grs::Vec3d e1;          // Polarization basis vectors
        grs::Vec3d e2;
        double amplitude = 0.0; // Fourier amplitude
        double phase = 0.0;     // Random phase
    };

    void m_generateModes();
    void m_calibrate();
    grs::Vec3d m_randomUnitVector();

    params m_parameters;

    std::vector<fourierMode> m_fourierModes;

    grs::Vec3d m_scaleFactor = {1.0, 1.0, 1.0};
    grs::Vec3d m_rmsUncalibrated = grs::Vec3d::zeros();

    // Random generator
    std::mt19937_64 m_rng;
    std::normal_distribution<double> m_normal{0.0,1.0};
    std::uniform_real_distribution<double> m_uniform{0.0,2.0*M_PI};
};

#endif //GRS_SIMULATOR_SPECTRALWINDFIELD_H