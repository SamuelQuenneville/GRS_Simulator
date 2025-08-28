/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "node.h"

int Node::m_idCounter = 0;

Node::Node()
    : m_id(++m_idCounter)
{

}

int Node::getId() const {
    return m_id;
}

void Node::update(const Vector3<double>& newPosition, const Vector3<double>& newVelocity, const Vector3<double>& newForce, const Vector3<double>& newMoment) {
    m_position = newPosition;
    m_velocity = newVelocity;
    m_force = newForce;
    m_moment = newMoment;
}

const Vector3<double>& Node::position() const {
    return m_position;
}

void Node::setPosition(const Vector3<double>& newPosition) {
    m_position = newPosition;
}

const Vector3<double>& Node::velocity() const {
    return m_velocity;
}

void Node::setVelocity(const Vector3<double>& newVelocity) {
    m_velocity = newVelocity;
}

const Vector3<double>& Node::acceleration() const {
    return m_acceleration;
}

void Node::setAcceleration(const Vector3<double>& newAcceleration) {
    m_acceleration = newAcceleration;
}

const Vector3<double>& Node::force() const {
    return m_force;
}

void Node::setForce(const Vector3<double>& newForce) {
    m_force = newForce;
}

const Vector3<double>& Node::moment() const {
    return m_moment;
}

void Node::setMoment(const Vector3<double>& newMoment) {
    m_moment = newMoment;
}
