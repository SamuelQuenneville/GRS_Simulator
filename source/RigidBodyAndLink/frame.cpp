/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */
#include <utility>

#include "frame.h"

Frame::Frame(const std::string& name, const Frame* reference, const Vector3<double>& position, const Vector3<double>& orientation)
    : m_localPosition(position)
    , m_localOrientation(orientation)
    , m_referenceOriginFrame(reference)
    , m_name(name)
{
    m_origin = std::make_unique<Node>();
    m_frameToBody = Matrix3<double>::fromEulerAngles(m_localOrientation);
}

void Frame::setLoadCallback(std::function<void(Frame &)> callback) {
    m_loadCallback = std::move(callback);
}

void Frame::computeLoad() {
    if (m_loadCallback) {
        m_loadCallback(*this);
    } else {
        setForce(Vector3<double>(0, 0, 0));
        setMoment(Vector3<double>(0, 0, 0));
    }
}

const Vector3<double>& Frame::getLocalPosition() const {
    return m_localPosition;
}

const Vector3<double>& Frame::getLocalOrientation() const {
    return m_localOrientation;
}

Node& Frame::originNode() const {
    return *m_origin;
}

void Frame::setTransform(const Transform& newTransform) {
    m_transform = newTransform;
}

void Frame::setTransform(const Vector3<double>& newPosition, const Quaternion& newOrientation) {
    m_transform.setPosition(newPosition);
    m_transform.setOrientation(newOrientation);
}

const Transform& Frame::transform() const {
    return m_transform;
}

