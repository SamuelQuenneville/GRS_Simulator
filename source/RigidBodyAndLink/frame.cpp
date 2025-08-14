/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "frame.h"

Frame::Frame(const Frame* reference, const Vector3<double>& position, const Vector3<double>& orientation)
    : m_localPosition(position)
    , m_localOrientation(orientation)
    , m_reference(reference)
{
    m_origin = std::make_shared<Node>();
}

const Vector3<double>& Frame::getLocalPosition() const {
    return m_localPosition;
}

const Vector3<double>& Frame::getLocalOrientation() const {
    return m_localOrientation;
}

const std::shared_ptr<Node>& Frame::origin() const {
    return m_origin;
}

