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
#include "Communication/interfaceJSON.h"


class System {

public:
    explicit System(programArguments& args, worldPhysics& worldPhysics, Logger& logger, systemConfiguration& configuration, systemProperties& properties, ctrlGains& gains);
    ~System();

    void update(double dt);
    void updateSitl(double dt);
    void setTarget(const target& newTarget) {m_target = newTarget;};
    target& getTarget() {return m_target;};

    void setControllerGains(const ctrlGains& newGains) {m_ctrlGains = newGains;};

    // Create the different configuration of system
    void createSystemOneAircraft(initialStates& aircraftStates);
    void createSystemOneAircraftGrounded(initialStates& aircraftStates);
    void createSystemTwoAircraft(std::vector<initialStates>& aircraftStates);
    void createSystemTwoAircraftV(bool isGrounded, std::vector<initialStates>& aircraftStates, initialStates& payloadStates);
    void createSystemTwoAircraftY();
    void createSystemThreeAircraftV(bool isGrounded, std::vector<initialStates>& aircraftStates, initialStates& payloadStates);

    std::vector<std::unique_ptr<FixedWing>>& aircraft() {return m_aircraft;};
    std::vector<std::unique_ptr<Tether>>& tethers() {return m_links;};
    std::unique_ptr<Payload>& payload() {return m_payload;};

    void setLaunchTriggered(const bool launchTriggered) {m_launchTriggered = launchTriggered;};

private:
    programArguments& m_programArgs;

    worldPhysics& m_physics;
    Logger& m_logger;
    std::unique_ptr<InterfaceJSON> m_communicationManager;

    systemConfiguration& m_systemConfiguration;
    systemProperties& m_systemProperties;
    ctrlGains& m_ctrlGains;

    bool m_launchTriggered = false;
    double m_launchStartMillis = 0;

    double m_timeSinceHlcCall;
    double m_timeSinceLaunch;

    bool m_shouldCallHlc();

    void m_initHlcCommands();

    target m_target{};
    std::map<int, fixedWingCommands> m_commands{};
    std::unique_ptr<HighLevelController<fixedWingCommands, fixedWingStates>> m_controller;

    std::vector<fixedWingStates> m_getAircraftStates() const;
    std::vector<int> m_getAircraftIds() const;

    void m_startUdpSocket(int uavId) const;

    std::vector<std::unique_ptr<FixedWing>> m_aircraft;
    std::vector<std::unique_ptr<Tether>> m_links;
    std::unique_ptr<Payload> m_payload;
};


#endif //GRS_SIMULATOR_SYSTEM_H
