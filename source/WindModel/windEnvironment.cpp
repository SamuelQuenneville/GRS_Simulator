/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "windEnvironment.h"

WindEnvironment::WindEnvironment(std::shared_ptr<MeanWindModel> meanWind, std::shared_ptr<TurbulenceModel> turbulence)
    : WindEnvironment(std::move(meanWind), std::move(turbulence), params())
{

}

WindEnvironment::WindEnvironment(std::shared_ptr<MeanWindModel> meanWind, std::shared_ptr<TurbulenceModel> turbulence, const params& params)
    : m_meanWind(std::move(meanWind))
    , m_turbulence(std::move(turbulence))
    , m_params(params)
{

}

grs::Vec3d WindEnvironment::getMeanWind(const grs::Vec3d& position) const {
    return m_meanWind->getMeanWind(position);
}

grs::Vec3d WindEnvironment::getTurbulence(const grs::Vec3d& position, const double time) const {
    const grs::Vec3d localMeanWind = m_meanWind->getMeanWind(position);
    const grs::Vec3d turb = m_turbulence->getTurbulence(position, localMeanWind, time);
    return turb * altitudeRamp(position[2]);
}

grs::Vec3d WindEnvironment::getTotalWind(const grs::Vec3d& position, const double time) const {
    return getMeanWind(position) + getTurbulence(position, time);
}

double WindEnvironment::altitudeRamp(const double z) const {
    const double h = m_params.altitudeSign * z;
    double t = (h - m_params.turbGroundHeight) / (m_params.turbFullHeight - m_params.turbGroundHeight);
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}