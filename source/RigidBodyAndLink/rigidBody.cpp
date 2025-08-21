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

std::map<int, Vector3<double>> RigidBody::externalForces() {
    std::map<int, Vector3<double>> forces;

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
            [&node](const Node *n) {
                return node.getId() == n->getId();
            }),
        linkEndConnections.end());
}

void RigidBody::updateForcesOnBody() {
    Vector3<double> totalForce = Vector3<double>::zeros();
    Vector3<double> totalMoment = Vector3<double>::zeros();

    for (const auto& frame : m_frames) {
        frame->computeLoad();

        Vector3<double> r = frame->getLocalPosition() - centerOfMassFrame()->getLocalPosition();

        totalForce += frame->forceBody();
        totalMoment += frame->momentBody() + r.cross(frame->forceBody());
    }

    centerOfMass().setForce(totalForce);
    centerOfMass().setMoment(totalMoment);
}

void RigidBody::updateWorldInfos(const Vector3<double>& position, const Vector3<double>& velocity, const Vector3<double>& acceleration) {
    centerOfMass().setPosition(position);
    centerOfMass().setVelocity(velocity);
    centerOfMass().setAcceleration(acceleration);

    for (const auto& f : m_frames) {
        if (!f->isCenterOfMass) {

            Vector3<double> pos_local = f->getLocalPosition() - centerOfMassFrame()->getLocalPosition();
            Vector3<double> pos_world = m_rotationMatrixCBodyEarth.transpose() * pos_local;

            Vector3<double> vel_frame = velocity + m_angularVelocity.cross(pos_world);
            Vector3<double> acc_frame = acceleration + m_angularAcceleration.cross(pos_world) + m_angularVelocity.cross(m_angularVelocity.cross(pos_world));

            f->originNode().setPosition(position + pos_world);
            f->originNode().setVelocity(vel_frame);
            f->originNode().setAcceleration(acc_frame);
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

void RigidBody::setAngularAcceleration(const Vector3<double>& newAngularAcceleration) {
    m_angularAcceleration = newAngularAcceleration;
}

void RigidBody::setRotationMatrixCBodyEarth(const Matrix3<double>& newRotationMatrix) {
    m_rotationMatrixCBodyEarth = newRotationMatrix;
}

const Vector3<double>& RigidBody::getAttitude() const {
    return m_attitude;
}

const Vector3<double>& RigidBody::getAngularVelocity() const {
    return m_angularVelocity;
}

const Vector3<double>& RigidBody::getAngularAcceleration() const {
    return m_angularAcceleration;
}

const Matrix3<double>& RigidBody::getRotationMatrixCBodyEarth() const {
    return m_rotationMatrixCBodyEarth;
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
