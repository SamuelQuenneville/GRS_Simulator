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

const Node& RigidBody::node() const {
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

