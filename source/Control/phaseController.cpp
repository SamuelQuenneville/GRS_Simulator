/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "phaseController.h"

PhaseController::PhaseController(worldPhysics& physics, float rate)
    : m_physics(physics)
    , m_dt(1.0f/rate)
{

}

float PhaseController::getThrustOut(float desiredPhase, bool disableIntegrator) {
    m_phasePid.setDt(m_dt);

    // reset I if required
    if (disableIntegrator){
        m_phasePid.resetI();
    }

    return 0;
}

void PhaseController::setAircraftStates(const fixedWingStates& aircraftStates) {
    m_aircraftStates = aircraftStates;
}
