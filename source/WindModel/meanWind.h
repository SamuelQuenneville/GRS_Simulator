/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_MEANWIND_H
#define GRS_SIMULATOR_MEANWIND_H

#include "meanWindModel.h"

// ===========================================================================
// MeanWind
//
// Concrete IMeanWindModel with three altitude profiles:
//
//   None      : constant with altitude
//   Ramp      : smoothstep from 0 at groundHeight to full meanWind at
//               fullHeight (matches the turbulence amplitude ramp in
//               WindEnvironment -- simple, consistent, good default)
//   PowerLaw  : classical power-law wind shear, U(h) = meanWind *
//               (h/fullHeight)^powerLawExponent -- the standard engineering
//               shear model (e.g. IEC 61400-1 uses alpha ~ 0.14-0.2; the
//               "one-seventh power law" alpha=1/7 is a common default for
//               open terrain). Unlike Ramp, this keeps increasing above
//               fullHeight, matching how real wind keeps strengthening with
//               altitude rather than plateauing.
//
// Physical note: unlike turbulence-amplitude suppression near the ground
// (a practical simplification -- see WindEnvironment), ramping the MEAN
// wind toward zero at the surface has a genuine physical basis: real mean
// wind goes to zero at the ground due to no-slip / surface friction.
// ===========================================================================
class MeanWind final : public MeanWindModel {
public:
    enum class Profile { None, Ramp, PowerLaw };

    struct params {
        grs::Vec3d meanWind{5.0, 0.0, 0.0};     // reference mean wind vector [m/s]
        Profile profile = Profile::Ramp;
        double powerLawExponent = 1.0 / 7.0;
        double groundHeight = 5.0;              // [m] altitude below which mean wind = 0
        double fullHeight   = 15.0;             // [m] altitude at/above which mean wind = full value
        double altitudeSign = -1.0;             // +1 if z increases with altitude (ENU) -1 for NED-style frames (z increases downward)
    };

    explicit MeanWind(const params& params);

    [[nodiscard]] grs::Vec3d getMeanWind(const grs::Vec3d& position) const override;

    // Exposed so other components (e.g. WindEnvironment's turbulence
    // amplitude ramp) can reuse the same smooth step shape if desired.
    [[nodiscard]] double altitudeFactor(double z) const;

    [[nodiscard]] const params& parameters() const { return m_params; }

private:
    params m_params;
};


#endif //GRS_SIMULATOR_MEANWIND_H