/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "payload.h"

Payload::Payload(const worldPhysics& physics, const payloadProperties& properties, const bool isGrounded)
    : m_physics(physics)
    , m_properties(properties)
    , m_isGrounded(isGrounded)
{
    // Create a cg Frame
    Frame& cg = addFrame(properties.name + "_centerOfMass", this->bodyOriginFrame(), {0,0,0}, {0,0,0}, Color::HexColor::WHITE);
    setCenterOfMassFrame(cg);

    // Create other Frames and assign function
    Frame& aeroReference = addFrame(properties.name + "_aeroRef", this->bodyOriginFrame(), {0,0,0}, {0,0,0}, Color::HexColor::CYAN);
    Frame& tetherAttach  = addFrame(properties.name + "_tetherAttach", this->bodyOriginFrame(), {0,0,0}, {0,0,0}, Color::HexColor::YELLOW);

    aeroReference.setLoadCallback([this](const Frame& f) {
        f.setForce(m_calcAeroForces());
        f.setMoment({0.0,0.0,0.0});
    });

    tetherAttach.setLoadCallback([this](const Frame& f) {
        grs::Vec3d tetherForces = grs::Vec3d::zeros();

        const auto connectedNode = connections[&f.originNode()];
        for(const Node* node : connectedNode) {
            tetherForces += node->force();
        }

        f.setForce(tetherForces);
        f.setMoment({0.0,0.0,0.0});
    });
}

void Payload::update(const double dt, const WindEnvironment& windEnv) {
    m_simTime += dt;

    m_wind = windEnv.getTotalWind(centerOfMass().position(), m_simTime);
    centerOfMass().setWind(m_wind);

    if (!m_isGrounded) {
        updateForcesOnBody();

        payloadStates derivatives = m_calcStatesDerivatives();
        m_applyGroundFriction(derivatives, dt);
        const bool groundContact = m_eulerIntegration(derivatives, dt);

        // Inertial acceleration in NED; the ground can only push up while in contact
        grs::Vec3d accelNed(derivatives.vx, derivatives.vy, derivatives.vz);
        if (groundContact) {
            accelNed[2] = std::min(accelNed[2], 0.0);
        }

        // Set new forces/moments to the node of this body
        updateWorldInfos({m_states.x, m_states.y, m_states.z}, {m_states.vx, m_states.vy, m_states.vz}, accelNed);
        setSpecificForceBody(m_calcSpecificForceBody(accelNed));
        setCenterOfMassTransform(grs::TransformD(centerOfMass().position().toOpenglRef(), grs::QuaternionD::identity()));
    } else {
        // Static body: an accelerometer at rest reads [0 0 -g]
        setSpecificForceBody(m_calcSpecificForceBody(grs::Vec3d::zeros()));
    }

    // Log
    m_writeLog();
}

int Payload::getId() const {
    return m_id;
}

void Payload::setId(const int id) {
    m_id = id;
}

void Payload::setInitialStates(initialStates& states) {
    m_states.x = states.position[0];
    m_states.y = states.position[1];
    m_states.z = states.position[2];

    m_states.vx = states.speed[0];
    m_states.vy = states.speed[1];
    m_states.vz = states.speed[2];

    updateWorldInfos({m_states.x, m_states.y, m_states.z}, {m_states.vx, m_states.vy, m_states.vz}, {0,0,0});
    setSpecificForceBody(m_calcSpecificForceBody(grs::Vec3d::zeros()));
    setCenterOfMassTransform(grs::TransformD(centerOfMass().position().toOpenglRef(), grs::eulerToQuat(states.orientation[0],states.orientation[1],states.orientation[2]).nedToYup()));
}

void Payload::m_writeLog() const {

    std::ostringstream msg;
    msg << m_simTime << "," << Logger::nowWallTimeMs() << "," << m_properties.name << ","
        << m_states.x  << "," << m_states.y  << "," << m_states.z  << ","
        << m_states.vx << "," << m_states.vy << "," << m_states.vz << ","
        << m_wind[0]   << "," << m_wind[1]   << "," << m_wind[2]   << ","
        << centerOfMass().acceleration()[0] << "," << centerOfMass().acceleration()[1] << "," << centerOfMass().acceleration()[2] << ","
        << getSpecificForceBody()[0] << "," << getSpecificForceBody()[1] << "," << getSpecificForceBody()[2];

    Logger::instance().log(LogType::Payload, msg.str());

    logFramesData(m_simTime);
}

payloadStates Payload::m_calcStatesDerivatives() const {
    payloadStates derivatives{};

    derivatives.x = m_states.vx;
    derivatives.y = m_states.vy;
    derivatives.z = m_states.vz;

    auto forces = centerOfMass().force();

    derivatives.vx  = forces[0] / m_properties.mass;
    derivatives.vy  = forces[1] / m_properties.mass;
    derivatives.vz  = (forces[2] + m_properties.mass * m_physics.gravity) / m_properties.mass;

    return derivatives;
}

bool Payload::m_eulerIntegration(const payloadStates& derivatives, const double dt) {

    m_states.x += derivatives.x * dt;
    m_states.y += derivatives.y * dt;
    m_states.z += derivatives.z * dt;

    m_states.vx += derivatives.vx * dt;
    m_states.vy += derivatives.vy * dt;
    m_states.vz += derivatives.vz * dt;

    // Simulate the ground plane at z = 0
    if (m_states.z >= 0.0 && m_states.vz > 0.0) { // penetrating and moving further down
        m_states.z  = 0.0;
        m_states.vz = 0.0;
        return true;
    }

    return false;
}

void Payload::m_applyGroundFriction(payloadStates& derivatives, const double dt) {
    // On the ground and pressed into it: the ground supplies the normal force.
    if (m_states.z < 0.0 || derivatives.vz <= 0.0) return;
    const double normal = m_properties.mass * derivatives.vz;

    const double v = std::hypot(m_states.vx, m_states.vy);
    const double a = std::hypot(derivatives.vx, derivatives.vy);
    constexpr double vStick = 1e-3; // [m/s]

    if (v < vStick && m_properties.mass * a <= m_properties.frictionStatic * normal) {
        m_states.vx = 0.0;
        m_states.vy = 0.0;
        derivatives.x = 0.0;
        derivatives.y = 0.0;
        derivatives.vx = 0.0;
        derivatives.vy = 0.0;
        return;
    }

    // Sliding: against the velocity, or against the applied force when starting.
    const double ux = v >= vStick ? m_states.vx / v : derivatives.vx / a;
    const double uy = v >= vStick ? m_states.vy / v : derivatives.vy / a;
    const double aFriction = m_properties.frictionKinetic * normal / m_properties.mass;
    derivatives.vx -= aFriction * ux;
    derivatives.vy -= aFriction * uy;

    // Friction stops the body, it never reverses it within a step.
    if (v >= vStick && (m_states.vx + derivatives.vx * dt) * ux + (m_states.vy + derivatives.vy * dt) * uy < 0.0) {
        derivatives.vx = -m_states.vx / dt;
        derivatives.vy = -m_states.vy / dt;
    }
}

grs::Vec3d Payload::m_calcSpecificForceBody(const grs::Vec3d& accelNed) const {
    return accelNed - grs::Vec3d(0.0, 0.0, m_physics.gravity);
}

grs::Vec3d Payload::m_calcAeroForces() const {

    // Drag = -coeff * ||v_rel|| * v_rel; same as ||v_rel||^2 * v_rel/||v_rel||
    const double coeff = 0.5 * m_physics.airDensity * m_properties.cda;
    const grs::Vec3d v_rel = centerOfMass().velocity() - m_wind;

    return -coeff * v_rel.norm() * v_rel;
}

bodyRenderData Payload::packRenderData() const {

    std::vector<frameRenderData> framesData;

    for (const auto& f : frames()) {
        framesData.push_back(f->packRenderData());
    }

    bodyRenderData data{.type = RenderObjectType::PAYLOAD, .id = m_id, .transform = transform(), .frames = framesData};
    return data;
}