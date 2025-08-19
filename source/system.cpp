/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "system.h"

uint64_t nowMicros() {
    const uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    return us;
}

uint64_t nowMillis() {
    const uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    return ms;
}

System::System(programArguments& args, worldPhysics& worldPhysics, Logger& logger, systemConfiguration& configuration, systemProperties& properties, ctrlGains& gains)
    : m_programArgs(args)
    , m_physics(worldPhysics)
    , m_logger(logger)
    , m_systemConfiguration(configuration)
    , m_systemProperties(properties)
    , m_ctrlGains(gains)
{
    m_communicationManager = std::make_unique<InterfaceJSON>();
    m_timeSinceHlcCall = 0.0;
    m_timeSinceLaunch = 0.0;
}

System::~System() {
    m_aircraft.clear();
    m_links.clear();
}

void System::update(const double dt) {
    // keep track of simulation time
    m_timeSinceHlcCall += dt;

    // Call HLC
    if (m_shouldCallHlc()) {
        if (m_payload != nullptr) {
            m_commands = m_controller->calculateInputs(m_getAircraftIds(), m_getAircraftStates(), m_payload->getStates(), m_target);
        } else {
            m_commands = m_controller->calculateInputs(m_getAircraftIds(), m_getAircraftStates(), m_target);
        }
    }

    // Call tether update()
    if (m_systemConfiguration.configuration != "FREE") {
        for(const auto& tether : m_links) {
            tether->update(dt);
        }
    }

    // Call fixedWing update()
    for(const auto& aircraft : m_aircraft) {
        aircraft->setCommands(m_commands[aircraft->getId()]);
        aircraft->update(dt);
    }

    // Call payload update()
    if (m_payload != nullptr) {
        m_payload->update(dt);
    }
}

void System::updateSitl(const double dt) {

    // Receive JSON SITL
    std::map<int, std::array<uint16_t, 16>> pwms;
    std::map<int, flags> flags;
    pwms.clear();
    m_communicationManager->receiveData(pwms, flags);

    // If all forced command for all UAV provide a endSim = true we stop the update
    bool shouldEndSimulation = true;

    for (const auto& [key, f] : flags) {
        if (!f.endSimulation) {
            shouldEndSimulation = false;
            break;
        }
    }

    if (shouldEndSimulation) {
        if (m_pauseTimer) m_pauseTimer();
    }

    if (m_communicationManager->fcuOnline()) {
        // Call tether update()
        if (m_systemConfiguration.configuration != "FREE") {
            for(const auto& tether : m_links) {
                tether->update(dt);
            }
        }

        // Call fixedWing update()
        for(const auto& aircraft : m_aircraft) {

            // Are we on a launcher?
            if (m_systemConfiguration.haveLauncher) {

                // The launch was triggered
                if (m_launchTriggered) {
                    if (m_timeSinceLaunch == 0) {
                        aircraft->setLaunchTriggered(true);
                    }

                    aircraft->setInLaunch(true);

                    // The aircraft exited the launcher (timeSinceLaunch sec to msec)
                    if (m_timeSinceLaunch * 1000 > m_systemProperties.launcher.launchDuration) {
                        aircraft->setInLaunch(false);
                    }
                }
            }

            aircraft->updateSitl(dt, pwms[aircraft->getId()], flags[aircraft->getId()].shouldMove);

            // Send JSON SITL
            sensorData sensorData = {static_cast<double>(nowMicros()) / 1000000.0, aircraft->getAngularVelocity(), aircraft->centerOfMass().acceleration(), aircraft->centerOfMass().position(), aircraft->getAttitude(), aircraft->centerOfMass().velocity(), aircraft->airspeed(), aircraft->getRpm()};
            m_communicationManager->sendData(aircraft->getId(), sensorData);

            m_communicationManager->resetSyncFlags();
        }

        if (m_launchTriggered) {
            m_timeSinceLaunch += dt;
        }

        // Call payload update()
        if (m_payload != nullptr) {
            m_payload->update(dt);
        }
    }
}

void System::setPauseTimerCallback(std::function<void()> callback) {
    m_pauseTimer = std::move(callback);
}

void System::createSystemOneAircraft(initialStates& aircraftStates) {

    if (m_programArgs.sitlMode) {
        // Create aircraft and set initial conditions
        auto aircraft = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing);
        aircraft->setInitialStates(aircraftStates);
        aircraft->setLauncherAccel(m_systemProperties.launcher.launcherAccel);

        if (m_programArgs.inFlightLaunch) {
            aircraft->setInFlightLaunch();
        }

        m_startUdpSocket(aircraft->getId());

        // Push back Bodies into list
        m_aircraft.push_back(std::move(aircraft));

    } else {
        // Create HLC
        m_controller = std::make_unique<HlcSingleAircraft>(m_physics, m_logger, m_ctrlGains.hlc, m_systemConfiguration.hlcRate);

        // Create LLC
        std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);

        // Create aircraft and set initial conditions
        auto aircraft = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing, std::move(lowLevelController));
        aircraft->setInitialStates(aircraftStates);

        // Push back Bodies into list
        m_aircraft.push_back(std::move(aircraft));

        // Compute first HLC commands
        m_initHlcCommands();
    }
}

void System::createSystemOneAircraftGrounded(initialStates& aircraftStates) {

    if (m_programArgs.sitlMode) {
        // Create aircraft and set initial conditions
        auto aircraft = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing);
        aircraft->setInitialStates(aircraftStates);
        aircraft->setLauncherAccel(m_systemProperties.launcher.launcherAccel);

        if (m_programArgs.inFlightLaunch) {
            aircraft->setInFlightLaunch();
        }

        // Create tether and set fix to ground
        auto tether = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);
        tether->grounded();

        // Create connections
        aircraft->addConnection(tether->endPointNode());
        tether->addConnection(tether->endPointNode(), aircraft->getFrame("tetherAttach")->originNode());

        m_startUdpSocket(aircraft->getId());

        // Push back Bodies and Links into lists
        m_aircraft.push_back(std::move(aircraft));
        m_links.push_back(std::move(tether));

    } else {
        // Create HLC
        m_controller = std::make_unique<HlcSingleAircraft>(m_physics, m_logger, m_ctrlGains.hlc, m_systemConfiguration.hlcRate);

        // Create LLC
        std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);

        // Create aircraft and set initial conditions
        auto aircraft = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing, std::move(lowLevelController));
        aircraft->setInitialStates(aircraftStates);

        // Create tether and set fix to ground
        auto tether = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);
        tether->grounded();

        // Create connections
        aircraft->addConnection(tether->endPointNode());
        tether->addConnection(tether->endPointNode(), aircraft->getFrame("tetherAttach")->originNode());

        // Push back Bodies and Links into lists
        m_aircraft.push_back(std::move(aircraft));
        m_links.push_back(std::move(tether));

        // Compute first HLC commands
        m_initHlcCommands();
    }
}

void System::createSystemTwoAircraft(std::vector<initialStates>& aircraftStates) {

    if (m_programArgs.sitlMode) {
        // Create aircraft and set initial conditions
        auto aircraft1 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing);
        aircraft1->setInitialStates(aircraftStates.front());
        aircraft1->setLauncherAccel(m_systemProperties.launcher.launcherAccel);

        auto aircraft2 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing);
        aircraft2->setInitialStates(aircraftStates.back());
        aircraft2->setLauncherAccel(m_systemProperties.launcher.launcherAccel);

        if (m_programArgs.inFlightLaunch) {
            aircraft1->setInFlightLaunch();
            aircraft2->setInFlightLaunch();
        }

        // Push back Bodies and Links into lists
        m_aircraft.push_back(std::move(aircraft1));
        m_aircraft.push_back(std::move(aircraft2));

    } else {
        // Create HLC
        m_controller = std::make_unique<HlcTwinAircraft>(m_physics, m_logger, m_ctrlGains.hlc, m_systemConfiguration.hlcRate, m_systemProperties.tether.length);

        // Create LLC
        std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController1 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);
        std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController2 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);

        // Create aircraft and set initial conditions
        auto aircraft1 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing, std::move(lowLevelController1));
        aircraft1->setInitialStates(aircraftStates.front());

        auto aircraft2 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing, std::move(lowLevelController2));
        aircraft2->setInitialStates(aircraftStates.back());

        // Push back Bodies and Links into lists
        m_aircraft.push_back(std::move(aircraft1));
        m_aircraft.push_back(std::move(aircraft2));

        // Compute first HLC commands
        m_initHlcCommands();
    }
}

void System::createSystemTwoAircraftV(bool isGrounded, std::vector<initialStates>& aircraftStates, initialStates& payloadStates) {

    if (m_programArgs.sitlMode) {
        // Create aircraft and set initial conditions
        auto aircraft1 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing);
        aircraft1->setInitialStates(aircraftStates.front());
        aircraft1->setLauncherAccel(m_systemProperties.launcher.launcherAccel);

        auto aircraft2 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing);
        aircraft2->setInitialStates(aircraftStates.back());
        aircraft2->setLauncherAccel(m_systemProperties.launcher.launcherAccel);

        if (m_programArgs.inFlightLaunch) {
            aircraft1->setInFlightLaunch();
            aircraft2->setInFlightLaunch();
        }

        // Create payload and set initial conditions
        m_payload = std::make_unique<Payload>(m_physics, m_logger, m_systemProperties.payload, isGrounded);
        m_payload->setInitialStates(payloadStates);

        // Create tethers
        auto tether1 = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);
        auto tether2 = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);

        // Create connections
        aircraft1->addConnection(tether1->endPointNode());
        tether1->addConnection(tether1->endPointNode(), aircraft1->getFrame("tetherAttach")->originNode());

        aircraft2->addConnection(tether2->endPointNode());
        tether2->addConnection(tether2->endPointNode(), aircraft2->getFrame("tetherAttach")->originNode());

        m_payload->addConnection(tether1->startPointNode());
        tether1->addConnection(tether1->startPointNode(), m_payload->centerOfMass());

        m_payload->addConnection(tether2->startPointNode());
        tether2->addConnection(tether2->startPointNode(),m_payload->centerOfMass());

        m_startUdpSocket(aircraft1->getId());
        m_startUdpSocket(aircraft2->getId());

        // Push back Bodies and Links into lists
        m_aircraft.push_back(std::move(aircraft1));
        m_aircraft.push_back(std::move(aircraft2));
        m_links.push_back(std::move(tether1));
        m_links.push_back(std::move(tether2));

    } else {
        // Create HLC
        m_controller = std::make_unique<HlcTwinAircraft>(m_physics, m_logger, m_ctrlGains.hlc, m_systemConfiguration.hlcRate, m_systemProperties.tether.length);

        // Create LLC
        std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController1 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);
        std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController2 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);

        // Create aircraft and set initial conditions
        auto aircraft1 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing, std::move(lowLevelController1));
        aircraft1->setInitialStates(aircraftStates.front());

        auto aircraft2 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing, std::move(lowLevelController2));
        aircraft2->setInitialStates(aircraftStates.back());

        // Create payload and set initial conditions
        m_payload = std::make_unique<Payload>(m_physics, m_logger, m_systemProperties.payload, isGrounded);
        m_payload->setInitialStates(payloadStates);

        // Create tethers and set fix to ground
        auto tether1 = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);
        auto tether2 = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);
        //
        // Create connections
        aircraft1->addConnection(tether1->endPointNode());
        tether1->addConnection(tether1->endPointNode(), aircraft1->getFrame("tetherAttach")->originNode());

        aircraft2->addConnection(tether2->endPointNode());
        tether2->addConnection(tether2->endPointNode(), aircraft2->getFrame("tetherAttach")->originNode());

        m_payload->addConnection(tether1->startPointNode());
        tether1->addConnection(tether1->startPointNode(), m_payload->centerOfMass());

        m_payload->addConnection(tether2->startPointNode());
        tether2->addConnection(tether2->startPointNode(), m_payload->centerOfMass());

        // Push back Bodies and Links into lists
        m_aircraft.push_back(std::move(aircraft1));
        m_aircraft.push_back(std::move(aircraft2));
        m_links.push_back(std::move(tether1));
        m_links.push_back(std::move(tether2));

        // Compute first HLC commands
        m_initHlcCommands();
    }
}

void System::createSystemTwoAircraftY() {

}

void System::createSystemThreeAircraftV(bool isGrounded, std::vector<initialStates>& aircraftStates, initialStates& payloadStates) {

    if (m_programArgs.sitlMode) {
        // Create aircraft and set initial conditions
        auto aircraft1 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing);
        aircraft1->setInitialStates(aircraftStates.at(0));

        auto aircraft2 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing);
        aircraft2->setInitialStates(aircraftStates.at(1));

        auto aircraft3 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing);
        aircraft3->setInitialStates(aircraftStates.at(2));

        if (m_programArgs.inFlightLaunch) {
            aircraft1->setInFlightLaunch();
            aircraft2->setInFlightLaunch();
            aircraft3->setInFlightLaunch();
        }

        // Create payload and set initial conditions
        m_payload = std::make_unique<Payload>(m_physics, m_logger, m_systemProperties.payload, isGrounded);
        m_payload->setInitialStates(payloadStates);

        // Create tethers and set fix to ground
        auto tether1 = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);
        auto tether2 = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);
        auto tether3 = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);

        // Create connections
        aircraft1->addConnection(tether1->endPointNode());
        tether1->addConnection(tether1->endPointNode(), aircraft1->getFrame("tetherAttach")->originNode());

        aircraft2->addConnection(tether2->endPointNode());
        tether2->addConnection(tether2->endPointNode(), aircraft2->getFrame("tetherAttach")->originNode());

        aircraft3->addConnection(tether3->endPointNode());
        tether3->addConnection(tether3->endPointNode(), aircraft3->getFrame("tetherAttach")->originNode());

        m_payload->addConnection(tether1->startPointNode());
        tether1->addConnection(tether1->startPointNode(), m_payload->centerOfMass());

        m_payload->addConnection(tether2->startPointNode());
        tether2->addConnection(tether2->startPointNode(), m_payload->centerOfMass());

        m_payload->addConnection(tether3->startPointNode());
        tether3->addConnection(tether3->startPointNode(), m_payload->centerOfMass());

        // Push back Bodies and Links into lists
        m_aircraft.push_back(std::move(aircraft1));
        m_aircraft.push_back(std::move(aircraft2));
        m_aircraft.push_back(std::move(aircraft3));
        m_links.push_back(std::move(tether1));
        m_links.push_back(std::move(tether2));
        m_links.push_back(std::move(tether3));

    } else {
        // Create HLC
        // TODO not a controller for 3 aircraft
        m_controller = std::make_unique<HlcTwinAircraft>(m_physics, m_logger, m_ctrlGains.hlc, m_systemConfiguration.hlcRate, m_systemProperties.tether.length);

        // Create LLC
        std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController1 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);
        std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController2 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);
        std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController3 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);

        // Create aircraft and set initial conditions
        auto aircraft1 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing, std::move(lowLevelController1));
        aircraft1->setInitialStates(aircraftStates.at(0));

        auto aircraft2 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing, std::move(lowLevelController2));
        aircraft2->setInitialStates(aircraftStates.at(1));

        auto aircraft3 = std::make_unique<FixedWing>(m_physics, m_logger, m_systemProperties.fixedWing, std::move(lowLevelController3));
        aircraft3->setInitialStates(aircraftStates.at(2));

        // Create payload and set initial conditions
        m_payload = std::make_unique<Payload>(m_physics, m_logger, m_systemProperties.payload, isGrounded);
        m_payload->setInitialStates(payloadStates);

        // Create tethers and set fix to ground
        auto tether1 = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);
        auto tether2 = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);
        auto tether3 = std::make_unique<Tether>(m_physics, m_logger, m_systemProperties.tether);

        // Create connections
        aircraft1->addConnection(tether1->endPointNode());
        tether1->addConnection(tether1->endPointNode(), aircraft1->getFrame("tetherAttach")->originNode());

        aircraft2->addConnection(tether2->endPointNode());
        tether2->addConnection(tether2->endPointNode(), aircraft2->getFrame("tetherAttach")->originNode());

        aircraft3->addConnection(tether3->endPointNode());
        tether3->addConnection(tether3->endPointNode(), aircraft3->getFrame("tetherAttach")->originNode());

        m_payload->addConnection(tether1->startPointNode());
        tether1->addConnection(tether1->startPointNode(), m_payload->centerOfMass());

        m_payload->addConnection(tether2->startPointNode());
        tether2->addConnection(tether2->startPointNode(), m_payload->centerOfMass());

        m_payload->addConnection(tether3->startPointNode());
        tether3->addConnection(tether3->startPointNode(), m_payload->centerOfMass());

        // Push back Bodies and Links into lists
        m_aircraft.push_back(std::move(aircraft1));
        m_aircraft.push_back(std::move(aircraft2));
        m_aircraft.push_back(std::move(aircraft3));
        m_links.push_back(std::move(tether1));
        m_links.push_back(std::move(tether2));
        m_links.push_back(std::move(tether3));

        // Compute first HLC commands
        m_initHlcCommands();
    }
}

std::vector<fixedWingStates> System::m_getAircraftStates() const {
    std::vector<fixedWingStates> statesVector{};

    for(const auto& aircraft : m_aircraft) {
        statesVector.push_back(aircraft->getStates());
    }
    return statesVector;
}

bool System::m_shouldCallHlc() {
    if (m_timeSinceHlcCall >= 1.0 / static_cast<double>(m_systemConfiguration.hlcRate)) {
        m_timeSinceHlcCall = 0.0;
        return true;
    }
    return false;
}

void System::m_initHlcCommands() {
    m_commands = m_controller->calculateInputs(m_getAircraftIds(), m_getAircraftStates(), m_target);
}

std::vector<int> System::m_getAircraftIds() const {
    std::vector<int> idsList{};

    for(const auto& aircraft : m_aircraft) {
        idsList.push_back(aircraft->getId());
    }
    return idsList;
}

void System::m_startUdpSocket(const int uavId) const {
    if (m_communicationManager->addSocket(uavId, "127.0.0.1", 9002 + 10*(uavId-1))) {
        LOG_INFO("Started socket --> 127.0.0.1:" + std::to_string(9002 + 10*(uavId-1)));
    }
}
