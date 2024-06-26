/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "link.h"

Link::Link() {
    startPoint = std::make_unique<Node>();
    endPoint = std::make_unique<Node>();
}

Node& Link::startPointNode() const {
    return *startPoint;
}

Node& Link::endPointNode() const {
    return *endPoint;
}

void Link::addConnection(Node& linkEnd, Node& node) {
    connections[&linkEnd].push_back(&node);
    linkEnd.setPosition(node.position());
    linkEnd.setVelocity(node.velocity());
}

void Link::deleteConnection(Node& linkEnd, Node& node) {
    std::vector<Node*> &linkEndConnections = connections[&linkEnd];
    linkEndConnections.erase(std::remove_if(linkEndConnections.begin(), linkEndConnections.end(), [node](Node *n){return node.getId() == n->getId();}), linkEndConnections.end());
}

