/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "rigidBody.h"

RigidBody::RigidBody() {
    centerOfMass = std::make_unique<Node>();
}

Node& RigidBody::node() {
    return *centerOfMass;
}

Vector3<double> RigidBody::calcExternalForce() {
    Vector3<double> sumExternalForce = {0,0,0};

    for(const Node* node : connectedNode) {
        sumExternalForce += node->force();
    }

    return sumExternalForce;
}

void RigidBody::addConnection(Node& node) {
    connectedNode.push_back(&node);
}

void RigidBody::deleteConnection(Node& node) {
    connectedNode.erase(std::remove_if(connectedNode.begin(), connectedNode.end(), [node](Node* n){return node.getId() == n->getId();}), connectedNode.end());
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

void RigidBody::setVelocity(const Vector3<double>& newVelocity) const {
    centerOfMass->setVelocity(newVelocity);
}

void RigidBody::setAngularVelocity(const Vector3<double>& newAngularVelocity) {
    m_angularVelocity = newAngularVelocity;
}

const Vector3<double> &RigidBody::getAngularVelocity() {
    return m_angularVelocity;
}
