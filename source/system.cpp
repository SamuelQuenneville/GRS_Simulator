/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "system.h"

uint64_t micros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

System::System(programArguments& args, worldPhysics& worldPhysics, Logger& logger, systemConfiguration& configuration, ctrlGains& gains)
    : m_programArgs(args)
    , m_physics(worldPhysics)
    , m_logger(logger)
    , m_systemConfiguration(configuration)
    , m_ctrlGains(gains)
{
    m_communicationManager = std::make_unique<InterfaceJSON>();
    m_timeSinceHlcCall = 0.0;
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
    // keep track of simulation time
    m_timeSinceHlcCall += dt;

    // Receive JSON SITL
    std::map<int, std::array<uint16_t, 16>> data;
    data.clear();
    if (m_communicationManager->receiveData(data)) {}

    if (m_communicationManager->fcuOnline()) {
        // Call tether update()
        if (m_systemConfiguration.configuration != "FREE") {
            for(const auto& tether : m_links) {
                tether->update(dt);
            }
        }

        // Call fixedWing update()
        for(const auto& aircraft : m_aircraft) {
            aircraft->updateSitl(dt, data[aircraft->getId()]);

            // Send JSON SITL
            sensorData sensorData = {static_cast<double>(micros()) / 1000000.0, aircraft->getAngularVelocity(), aircraft->centerOfMass->acceleration(), aircraft->centerOfMass->position(), aircraft->getAttitude(), aircraft->centerOfMass->velocity(), aircraft->airspeed()};
            m_communicationManager->sendData(*m_communicationManager->getSockets()[aircraft->getId()], sensorData);
        }

        // Call payload update()
        if (m_payload != nullptr) {
            m_payload->update(dt);
        }
    }
}

void System::createSystemOneAircraft(fixedWingProperties& aircraftProperties, initialStates& aircraftStates) {

    if (m_programArgs.sitlMode) {
        // Create aircraft and set initial conditions
        std::unique_ptr<FixedWing> aircraft = std::make_unique<FixedWing>(m_physics, m_logger, aircraftProperties);
        aircraft->setInitialStates(aircraftStates);

        m_startUdpSocket(aircraft->getId());

        // Push back Bodies into list
        m_aircraft.push_back(std::move(aircraft));

    } else {
        // Create HLC
        m_controller = std::make_unique<HlcSingleAircraft>(m_physics, m_logger, m_ctrlGains.hlc, m_systemConfiguration.hlcRate);

        // Create LLC
        std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);

        // Create aircraft and set initial conditions
        std::unique_ptr<FixedWing> aircraft = std::make_unique<FixedWing>(m_physics, m_logger, aircraftProperties, std::move(lowLevelController));
        aircraft->setInitialStates(aircraftStates);

        // Push back Bodies into list
        m_aircraft.push_back(std::move(aircraft));

        // Compute first HLC commands
        m_initHlcCommands();
    }
}

void System::createSystemOneAircraftGrounded(fixedWingProperties& aircraftProperties, tetherProperties& tetherProperties, initialStates& aircraftStates) {
    // Create HLC
    m_controller = std::make_unique<HlcSingleAircraft>(m_physics, m_logger, m_ctrlGains.hlc, m_systemConfiguration.hlcRate);

    // Create LLC
    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);

    // Create aircraft and set initial conditions
    std::unique_ptr<FixedWing> aircraft = std::make_unique<FixedWing>(m_physics, m_logger, aircraftProperties, std::move(lowLevelController));
    aircraft->setInitialStates(aircraftStates);

    // Create tether and set fix to ground
    std::unique_ptr<Tether> tether = std::make_unique<Tether>(m_physics, m_logger, tetherProperties);
    tether->grounded();

    // Create connections
    aircraft->addConnection(tether->endPointNode());
    tether->addConnection(tether->endPointNode(), aircraft->node());

    // Push back Bodies and Links into lists
    m_aircraft.push_back(std::move(aircraft));
    m_links.push_back(std::move(tether));

    // Compute first HLC commands
    m_initHlcCommands();
}

void System::createSystemTwoAircraft(fixedWingProperties& aircraftProperties, std::vector<initialStates>& aircraftStates) {
    // Create HLC
    m_controller = std::make_unique<HlcTwinAircraft>(m_physics, m_logger, m_ctrlGains.hlc, m_systemConfiguration.hlcRate, 30.0);

    // Create LLC
    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController1 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);
    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController2 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);

    // Create aircraft and set initial conditions
    std::unique_ptr<FixedWing> aircraft1 = std::make_unique<FixedWing>(m_physics, m_logger, aircraftProperties, std::move(lowLevelController1));
    aircraft1->setInitialStates(aircraftStates.front());

    std::unique_ptr<FixedWing> aircraft2 = std::make_unique<FixedWing>(m_physics, m_logger, aircraftProperties, std::move(lowLevelController2));
    aircraft2->setInitialStates(aircraftStates.back());

    // Push back Bodies and Links into lists
    m_aircraft.push_back(std::move(aircraft1));
    m_aircraft.push_back(std::move(aircraft2));

    // Compute first HLC commands
    m_initHlcCommands();
}

void System::createSystemTwoAircraftV(bool isGrounded, fixedWingProperties& aircraftProperties, tetherProperties& tetherProperties, payloadProperties& payloadProperties, std::vector<initialStates>& aircraftStates, initialStates& payloadStates) {
    // Create HLC
    m_controller = std::make_unique<HlcTwinAircraft>(m_physics, m_logger, m_ctrlGains.hlc, m_systemConfiguration.hlcRate, tetherProperties.length);

    // Create LLC
    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController1 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);
    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController2 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);

    // Create aircraft and set initial conditions
    std::unique_ptr<FixedWing> aircraft1 = std::make_unique<FixedWing>(m_physics, m_logger, aircraftProperties, std::move(lowLevelController1));
    aircraft1->setInitialStates(aircraftStates.front());

    std::unique_ptr<FixedWing> aircraft2 = std::make_unique<FixedWing>(m_physics, m_logger, aircraftProperties, std::move(lowLevelController2));
    aircraft2->setInitialStates(aircraftStates.back());

    // Create payload and set initial conditions
    m_payload = std::make_unique<Payload>(m_physics, m_logger, payloadProperties, isGrounded);
    m_payload->setInitialStates(payloadStates);

    // Create tethers and set fix to ground
    std::unique_ptr<Tether> tether1 = std::make_unique<Tether>(m_physics, m_logger, tetherProperties);
    std::unique_ptr<Tether> tether2 = std::make_unique<Tether>(m_physics, m_logger, tetherProperties);

    // Create connections
    aircraft1->addConnection(tether1->endPointNode());
    tether1->addConnection(tether1->endPointNode(), aircraft1->node());

    aircraft2->addConnection(tether2->endPointNode());
    tether2->addConnection(tether2->endPointNode(), aircraft2->node());

    m_payload->addConnection(tether1->startPointNode());
    tether1->addConnection(tether1->startPointNode(), m_payload->node());

    m_payload->addConnection(tether2->startPointNode());
    tether2->addConnection(tether2->startPointNode(), m_payload->node());

    // Push back Bodies and Links into lists
    m_aircraft.push_back(std::move(aircraft1));
    m_aircraft.push_back(std::move(aircraft2));
    m_links.push_back(std::move(tether1));
    m_links.push_back(std::move(tether2));

    // Compute first HLC commands
    m_initHlcCommands();
}

void System::createSystemTwoAircraftY() {

}

void System::createSystemThreeAircraftV(bool isGrounded, fixedWingProperties& aircraftProperties, tetherProperties& tetherProperties, payloadProperties& payloadProperties, std::vector<initialStates>& aircraftStates, initialStates& payloadStates) {
    // Create HLC
    // TODO not a controller for 3 aircraft
    m_controller = std::make_unique<HlcTwinAircraft>(m_physics, m_logger, m_ctrlGains.hlc, m_systemConfiguration.hlcRate, tetherProperties.length);

    // Create LLC
    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController1 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);
    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController2 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);
    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> lowLevelController3 = std::make_unique<LlcBaseVersion>(m_physics, m_logger, m_ctrlGains.llc, m_systemConfiguration.llcRate);

    // Create aircraft and set initial conditions
    std::unique_ptr<FixedWing> aircraft1 = std::make_unique<FixedWing>(m_physics, m_logger, aircraftProperties, std::move(lowLevelController1));
    aircraft1->setInitialStates(aircraftStates.at(0));

    std::unique_ptr<FixedWing> aircraft2 = std::make_unique<FixedWing>(m_physics, m_logger, aircraftProperties, std::move(lowLevelController2));
    aircraft2->setInitialStates(aircraftStates.at(1));

    std::unique_ptr<FixedWing> aircraft3 = std::make_unique<FixedWing>(m_physics, m_logger, aircraftProperties, std::move(lowLevelController3));
    aircraft3->setInitialStates(aircraftStates.at(2));

    // Create payload and set initial conditions
    m_payload = std::make_unique<Payload>(m_physics, m_logger, payloadProperties, isGrounded);
    m_payload->setInitialStates(payloadStates);

    // Create tethers and set fix to ground
    std::unique_ptr<Tether> tether1 = std::make_unique<Tether>(m_physics, m_logger, tetherProperties);
    std::unique_ptr<Tether> tether2 = std::make_unique<Tether>(m_physics, m_logger, tetherProperties);
    std::unique_ptr<Tether> tether3 = std::make_unique<Tether>(m_physics, m_logger, tetherProperties);

    // Create connections
    aircraft1->addConnection(tether1->endPointNode());
    tether1->addConnection(tether1->endPointNode(), aircraft1->node());

    aircraft2->addConnection(tether2->endPointNode());
    tether2->addConnection(tether2->endPointNode(), aircraft2->node());

    aircraft3->addConnection(tether3->endPointNode());
    tether3->addConnection(tether3->endPointNode(), aircraft3->node());

    m_payload->addConnection(tether1->startPointNode());
    tether1->addConnection(tether1->startPointNode(), m_payload->node());

    m_payload->addConnection(tether2->startPointNode());
    tether2->addConnection(tether2->startPointNode(), m_payload->node());

    m_payload->addConnection(tether3->startPointNode());
    tether3->addConnection(tether3->startPointNode(), m_payload->node());

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
