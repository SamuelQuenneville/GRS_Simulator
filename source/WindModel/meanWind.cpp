/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "meanWind.h"

MeanWind::MeanWind(const windProperties& params)
    : m_params(params)
{
    if (m_params.meanWindFullHeight <= m_params.meanWindGroundHeight) {
        throw std::invalid_argument("fullHeight must be greater than groundHeight");
    }
}

grs::Vec3d MeanWind::getMeanWind(const grs::Vec3d& position) const {
    return grs::toVector<double,3>(m_params.meanWind) * altitudeFactor(position[2]);
}

double MeanWind::altitudeFactor(const double z) const {
    const double h = std::abs(z);   // positive altitude

    if (m_params.meanWindPofile == "RAMP") {
        double t = (h - m_params.meanWindGroundHeight) / (m_params.meanWindFullHeight - m_params.meanWindGroundHeight);
        t = std::clamp(t, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);   // smooth step: continuous value AND slope
    }
    if (m_params.meanWindPofile == "POWERLAW") {
        if (h <= 0.0) return 0.0;
        return std::pow(h / m_params.meanWindFullHeight, 1.0/7.0);
    }

    return 1.0;
}
