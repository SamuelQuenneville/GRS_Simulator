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

}

Vector3<double> RigidBody::calcExternalForce() {
    Vector3<double> sumExternalForce = Vector3<double>::zeros();

    for(const Node* node : connectedNode) {
        sumExternalForce += node->force();
    }

    return sumExternalForce;
}

std::map<int, Vector3<double>> RigidBody::externalForces() {
    std::map<int, Vector3<double>> forces;

    for(const Node* node : connectedNode) {
        forces.emplace(node->getId(), node->force());
    }

    return forces;
}

void RigidBody::addConnection(Node& node) {
    connectedNode.push_back(&node);
}

void RigidBody::deleteConnection(Node& node) {
    connectedNode.erase(
        std::remove_if(
            connectedNode.begin(), connectedNode.end(),
            [&node](const Node* n) {
                return n != nullptr && node.getId() == n->getId();
            }),
        connectedNode.end());
}

void RigidBody::updateWorldInfos(const Vector3<double>& position, const Vector3<double>& velocity, const Vector3<double>& acceleration) const {
    centerOfMass().setPosition(position);
    centerOfMass().setVelocity(velocity);
    centerOfMass().setAcceleration(acceleration);

    for (const auto& f : m_frames) {
        if (!f->isCenterOfMass) {
            f->originNode().setPosition(position);
            f->originNode().setVelocity(velocity);
            f->originNode().setAcceleration(acceleration);
        }
    }

}

void RigidBody::setTransform(const Transform& newTransform) {
    m_transform = newTransform;
}

void RigidBody::setTransform(const Vector3<double>& newPosition, const Quaternion& newOrientation) {
    m_transform.setPosition(newPosition);
    m_transform.setOrientation(newOrientation);
}

const Transform& RigidBody::transform() const {
    return m_transform;
}

void RigidBody::setAttitude(const Vector3<double>& newAttitude) {
    m_attitude = newAttitude;
}

void RigidBody::setAngularVelocity(const Vector3<double>& newAngularVelocity) {
    m_angularVelocity = newAngularVelocity;
}

const Vector3<double>& RigidBody::getAttitude() const {
    return m_attitude;
}

const Vector3<double>& RigidBody::getAngularVelocity() const {
    return m_angularVelocity;
}

Frame* RigidBody::centerOfMassFrame() const {
    for (const auto& f : m_frames) {
        if (f->isCenterOfMass == true) {
            return f.get();
        }
    }
    return nullptr;
}

Node& RigidBody::centerOfMass() const {
    if (centerOfMassFrame()) {
        return centerOfMassFrame()->originNode();
    }
    return m_bodyOriginFrame->originNode();
}

Frame& RigidBody::addFrame(const std::string& name, const Frame* reference, const Vector3<double>& position, const Vector3<double>& orientation) {
    auto frame = std::make_unique<Frame>(name, reference, position, orientation);
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
