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

void Node::update(const grs::Vec3d& newPosition, const grs::Vec3d& newVelocity, const grs::Vec3d& newForce, const grs::Vec3d& newMoment) {
    m_position = newPosition;
    m_velocity = newVelocity;
    m_force = newForce;
    m_moment = newMoment;
}

const grs::Vec3d& Node::position() const {
    return m_position;
}

void Node::setPosition(const grs::Vec3d& newPosition) {
    m_position = newPosition;
}

const grs::Vec3d& Node::velocity() const {
    return m_velocity;
}

void Node::setVelocity(const grs::Vec3d& newVelocity) {
    m_velocity = newVelocity;
}

const grs::Vec3d& Node::acceleration() const {
    return m_acceleration;
}

void Node::setAcceleration(const grs::Vec3d& newAcceleration) {
    m_acceleration = newAcceleration;
}

const grs::Vec3d& Node::force() const {
    return m_force;
}

void Node::setForce(const grs::Vec3d& newForce) {
    m_force = newForce;
}

const grs::Vec3d& Node::moment() const {
    return m_moment;
}

void Node::setMoment(const grs::Vec3d& newMoment) {
    m_moment = newMoment;
}

const grs::Vec3d& Node::wind() const {
    return m_wind;
}

void Node::setWind(const grs::Vec3d& newWind) {
    m_wind = newWind;
}
