/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "rigidBody.h"

RigidBody::RigidBody()
    : m_bodyOriginFrame(std::make_unique<Frame>())
{
    m_bodyOriginFrame->setIsCenterOfMass(true);
}

std::map<int, grs::Vec3d> RigidBody::externalForces() {
    std::map<int, grs::Vec3d> forces;

    for (auto& [key, neighbors] : connections) {
        for (const Node* node : neighbors) {
            forces.emplace(node->getId(), node->force());
        }
    }

    return forces;
}

void RigidBody::addConnection(Node& frameNode, Node& node) {
    connections[&frameNode].push_back(&node);
}

void RigidBody::deleteConnection(Node& frameNode, Node& node) {
    std::vector<Node*>& linkEndConnections = connections[&frameNode];
    linkEndConnections.erase(
        std::remove_if(
            linkEndConnections.begin(), linkEndConnections.end(),
            [&node](const Node* n) {
                return node.getId() == n->getId();
            }),
        linkEndConnections.end());
}

void RigidBody::logFramesData(const double simTime) const {
    for (const auto& frame : m_frames) {
        frame->writeLog(simTime);
    }
}

void RigidBody::updateForcesOnBody() {
    grs::Vec3d totalForce = grs::Vec3d::zeros();
    grs::Vec3d totalMoment = grs::Vec3d::zeros();

    for (const auto& frame : m_frames) {
        frame->computeLoad();

        grs::Vec3d r = frame->getLocalPosition() - centerOfMassFrame()->getLocalPosition();

        totalForce += frame->forceBody();
        totalMoment += frame->momentBody() + grs::cross(r, frame->forceBody());
    }

    centerOfMass().setForce(totalForce);
    centerOfMass().setMoment(totalMoment);
}

void RigidBody::updateWorldInfos(const grs::Vec3d& position, const grs::Vec3d& velocity, const grs::Vec3d& acceleration) const {
    centerOfMass().setPosition(position);
    centerOfMass().setVelocity(velocity);
    centerOfMass().setAcceleration(acceleration);

    for (const auto& f : m_frames) {
        if (!f->isCenterOfMass()) {

            grs::Vec3d pos_local = f->getLocalPosition() - centerOfMassFrame()->getLocalPosition();
            grs::Vec3d pos_world = m_rotationMatrixCBodyEarth.transpose() * pos_local;

            grs::Vec3d vel_frame = velocity + grs::cross(m_angularVelocity, pos_world);
            grs::Vec3d acc_frame = acceleration + grs::cross(m_angularAcceleration, pos_world) + grs::cross(m_angularVelocity, grs::cross(m_angularVelocity, pos_world));

            f->originNode().setPosition(position + pos_world);
            f->originNode().setVelocity(vel_frame);
            f->originNode().setAcceleration(acc_frame);
        }
    }
}

void RigidBody::setCenterOfMassTransform(const grs::TransformD& newTransform) const {
    centerOfMassFrame()->setTransform(newTransform);

    for (const auto& f : m_frames) {
        if (!f->isCenterOfMass()) {
            auto euler = f->getLocalOrientation();

            constexpr double meshRollOffset = -M_PI_2;
            auto cgToFrame = grs::TransformD(f->getLocalPosition().toOpenglRef(), grs::eulerToQuat(euler[0] + meshRollOffset, euler[1], euler[2]).nedToYup());

            auto earthToFrame = newTransform * cgToFrame;
            f->setTransform(earthToFrame);
        }
    }
}

void RigidBody::setTransform(const grs::TransformD& newTransform) const {
    centerOfMassFrame()->setTransform(newTransform);
}

void RigidBody::setTransform(const grs::Vec3d& newPosition, const grs::QuaternionD& newOrientation) const {
    centerOfMassFrame()->setTransform(newPosition, newOrientation);
}

const grs::TransformD& RigidBody::transform() const {
    return centerOfMassFrame()->transform();
}

void RigidBody::setAttitude(const grs::Vec3d& newAttitude) {
    m_attitude = newAttitude;
}

void RigidBody::setAngularVelocity(const grs::Vec3d& newAngularVelocity) {
    m_angularVelocity = newAngularVelocity;
}

void RigidBody::setAngularAcceleration(const grs::Vec3d& newAngularAcceleration) {
    m_angularAcceleration = newAngularAcceleration;
}

void RigidBody::setRotationMatrixCBodyEarth(const grs::Matrix3d& newRotationMatrix) {
    m_rotationMatrixCBodyEarth = newRotationMatrix;
}

const grs::Vec3d& RigidBody::getAttitude() const {
    return m_attitude;
}

const grs::Vec3d& RigidBody::getAngularVelocity() const {
    return m_angularVelocity;
}

const grs::Vec3d& RigidBody::getAngularAcceleration() const {
    return m_angularAcceleration;
}

const grs::Matrix3d& RigidBody::getRotationMatrixCBodyEarth() const {
    return m_rotationMatrixCBodyEarth;
}

Frame* RigidBody::centerOfMassFrame() const {
    for (const auto& f : m_frames) {
        if (f->isCenterOfMass()) {
            return f.get();
        }
    }
    return m_bodyOriginFrame.get();
}

Node& RigidBody::centerOfMass() const {
    if (centerOfMassFrame()) {
        return centerOfMassFrame()->originNode();
    }
    return m_bodyOriginFrame->originNode();
}

void RigidBody::setCenterOfMassFrame(Frame& frame) const {
    m_bodyOriginFrame->setIsCenterOfMass(false);
    frame.setIsCenterOfMass(true);
}

Frame& RigidBody::addFrame(const std::string& name, const Frame* reference, const grs::Vec3d& position, const grs::Vec3d& orientation, Color::HexColor color) {
    auto frame = std::make_unique<Frame>(name, reference, position, orientation, color);
    m_frames.push_back(std::move(frame));
    return *m_frames.back();
}

Frame* RigidBody::getFrame(const std::string& name) const {
    for (const auto& f : m_frames) {
        if (f->getName() == name) {
            return f.get();
        }
    }
    return nullptr;
}

std::vector<Frame*> RigidBody::frames() const {
    std::vector<Frame*> result;
    result.reserve(m_frames.size());
    for (auto& f : m_frames) {
        result.push_back(f.get());
    }
    return result;
}
