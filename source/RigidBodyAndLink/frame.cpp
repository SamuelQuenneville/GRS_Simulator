/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "frame.h"

Frame::Frame(const std::string& name, const Frame* reference, const grs::Vec3d& position, const grs::Vec3d& orientation, const Color::HexColor forceColor)
    : m_localPosition(position)
    , m_localOrientation(orientation)
    , m_referenceOriginFrame(reference)
    , m_name(name)
    , m_forceColor(forceColor)
{
    m_origin = std::make_unique<Node>();
    m_frameToBody = grs::eulerToMat(m_localOrientation);
}

void Frame::setLoadCallback(std::function<void(Frame&)> callback) {
    m_loadCallback = std::move(callback);
}

void Frame::computeLoad() {
    if (m_loadCallback) {
        m_loadCallback(*this);
    } else {
        setForce(grs::Vec3d::zeros());
        setMoment(grs::Vec3d::zeros());
    }
}

const grs::Vec3d& Frame::getLocalPosition() const {
    return m_localPosition;
}

const grs::Vec3d& Frame::getLocalOrientation() const {
    return m_localOrientation;
}

Node& Frame::originNode() const {
    return *m_origin;
}

void Frame::setTransform(const grs::TransformD& newTransform) {
    m_transform = newTransform;
}

void Frame::setTransform(const grs::Vec3d& newPosition, const grs::QuaternionD& newOrientation) {
    m_transform.position = newPosition;
    m_transform.orientation = newOrientation;
}

const grs::TransformD& Frame::transform() const {
    return m_transform;
}

