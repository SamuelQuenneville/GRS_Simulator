/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "meanWind.h"

MeanWind::MeanWind(const params& params)
    : m_params(params)
{
    if (m_params.fullHeight <= m_params.groundHeight) {
        throw std::invalid_argument("fullHeight must be greater than groundHeight");
    }
}

grs::Vec3d MeanWind::getMeanWind(const grs::Vec3d& position) const {
    return m_params.meanWind * altitudeFactor(position[2]);
}

double MeanWind::altitudeFactor(const double z) const {
    const double h = m_params.altitudeSign * z;   // positive-up altitude

    switch (m_params.profile) {
        case Profile::None:
            return 1.0;

        case Profile::Ramp: {
            double t = (h - m_params.groundHeight) / (m_params.fullHeight - m_params.groundHeight);
            t = std::clamp(t, 0.0, 1.0);
            return t * t * (3.0 - 2.0 * t);   // smooth step: continuous value AND slope
        }

        case Profile::PowerLaw: {
            if (h <= 0.0) return 0.0;
            return std::pow(h / m_params.fullHeight, m_params.powerLawExponent);
        }
    }
    return 1.0;  // unreachable
}