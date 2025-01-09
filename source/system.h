/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_SYSTEM_H
#define GRS_SIMULATOR_SYSTEM_H

#include "RigidBodyAndLink//fixedWing.h"
#include "RigidBodyAndLink/tether.h"
#include "RigidBodyAndLink/payload.h"

#include "Definitions/systemDefinitions.h"
#include "Definitions/controllerGains.h"
#include "Control/llcBaseVersion.h"
#include "Control/hlcSingleAircraft.h"
#include "Control/hlcTwinAircraft.h"


class System {

public:
    explicit System(worldPhysics& worldPhysics, Logger& logger, systemConfiguration& configuration, ctrlGains& gains);
    ~System();

    void update(double dt);
    void setTarget(const target& newTarget) {m_target = newTarget;};
    target& getTarget() {return m_target;};

    void setControllerGains(const ctrlGains& newGains) {m_ctrlGains = newGains;};

    // Create the different configuration of system
    void createSystemOneAircraft(fixedWingProperties& aircraftProperties, initialStates& aircraftStates);
    void createSystemOneAircraftGrounded(fixedWingProperties& aircraftProperties, tetherProperties& tetherProperties, initialStates& aircraftStates);
    void createSystemTwoAircraft(fixedWingProperties& aircraftProperties, std::vector<initialStates>& aircraftStates);
    void createSystemTwoAircraftV(bool isGrounded, fixedWingProperties& aircraftProperties, tetherProperties& tetherProperties, payloadProperties& payloadProperties, std::vector<initialStates>& aircraftStates, initialStates& payloadStates);
    void createSystemTwoAircraftY();
    void createSystemThreeAircraftV(bool isGrounded, fixedWingProperties& aircraftProperties, tetherProperties& tetherProperties, payloadProperties& payloadProperties, std::vector<initialStates>& aircraftStates, initialStates& payloadStates);

    std::vector<std::unique_ptr<FixedWing>>& aircraft() {return m_aircraft;};
    std::vector<std::unique_ptr<Tether>>& tethers() {return m_links;};
    std::unique_ptr<Payload>& payload() {return m_payload;};

private:
    worldPhysics& m_physics;
    Logger& m_logger;
    systemConfiguration& m_systemConfiguration;
    ctrlGains& m_ctrlGains;

    double m_timeSinceHlcCall;
    bool m_shouldCallHlc();

    void m_initHlcCommands();

    target m_target{};
    std::map<int, fixedWingCommands> m_commands{};
    std::unique_ptr<HighLevelController<fixedWingCommands, fixedWingStates>> m_controller;

    std::vector<fixedWingStates> m_getAircraftStates();
    std::vector<int> m_getAircraftIds();

    std::vector<std::unique_ptr<FixedWing>> m_aircraft;
    std::vector<std::unique_ptr<Tether>> m_links;
    std::unique_ptr<Payload> m_payload;
};


#endif //GRS_SIMULATOR_SYSTEM_H
