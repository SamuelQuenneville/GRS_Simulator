/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_WINDENVIRONMENT_H
#define GRS_SIMULATOR_WINDENVIRONMENT_H

#include <memory>

#include "Mathematics/math.h"
#include "spectralWindField.h"
#include "meanWind.h"

/*
 * WindEnvironment
 *
 * Mission/scenario-level wind model: composes a mean-wind model and a
 * turbulence model (through their abstract interfaces, not concrete types --
 * so either can be swapped later, e.g. a different shear profile or a
 * different turbulence spectrum, without touching this class).
 *
 * Owns the turbulence amplitude ramp -- suppressing turbulence near the
 * ground for e.g. a catapult/rail launch. This is a PRACTICAL simulation
 * simplification, not a claim about real atmospheric physics: near-surface
 * turbulence in the real ABL is usually *higher*, not lower, due to
 * mechanical generation by ground roughness. It lives here (not in
 * SpectralWindField) because it's a scenario-specific choice, not a
 * property of turbulence physics itself.
 */

class WindEnvironment {

public:

    struct params {
        double turbGroundHeight = 5.0;   // [m] altitude below which turbulence = 0
        double turbFullHeight   = 15.0;  // [m] altitude above which turbulence = full sigma
        double altitudeSign     = -1.0;  // +1 ENU (z up), -1 NED (z increases downward)
    };

    WindEnvironment(std::shared_ptr<MeanWindModel> meanWind, std::shared_ptr<TurbulenceModel> turbulence);
    WindEnvironment(std::shared_ptr<MeanWindModel> meanWind, std::shared_ptr<TurbulenceModel> turbulence, const params& params);

    // Mean wind alone, at this position (delegates to the mean-wind model)
    [[nodiscard]] grs::Vec3d getMeanWind(const grs::Vec3d &position) const;

    // Turbulence alone, with the altitude amplitude ramp applied, and
    // advection consistent with the LOCAL mean wind at that position (so
    // the turbulence pattern advects slower near the ground, matching the
    // calmer local wind, instead of silently using a fixed reference speed
    // everywhere).
    [[nodiscard]] grs::Vec3d getTurbulence(const grs::Vec3d& position, double time) const;

    // Convenience: mean wind + turbulence in one call
    [[nodiscard]] grs::Vec3d getTotalWind(const grs::Vec3d& position, double time) const;

    // Smooth step ramp: 0 below turbGroundHeight, 1 above turbFullHeight,
    // continuous value AND slope in between (a hard cutoff would inject a
    // discontinuous derivative into the gust signal right as the vehicle
    // clears the launcher).
    [[nodiscard]] double altitudeRamp(double z) const;

    [[nodiscard]] std::shared_ptr<MeanWindModel> meanWindModel() const { return m_meanWind; }
    [[nodiscard]] std::shared_ptr<TurbulenceModel> turbulenceModel() const { return m_turbulence; }

private:

    std::shared_ptr<MeanWindModel> m_meanWind;
    std::shared_ptr<TurbulenceModel> m_turbulence;
    params m_params;
};

#endif //GRS_SIMULATOR_WINDENVIRONMENT_H