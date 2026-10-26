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

void Frame::writeLog(const double simTime) const {
    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4)
        << simTime << "," << Logger::nowWallTimeMs() << "," << m_name << "," << m_origin->getId() << ","
        << m_origin->position()[0] << "," << m_origin->position()[1] << "," << m_origin->position()[2] << ","
        << m_origin->velocity()[0] << "," << m_origin->velocity()[1] << "," << m_origin->velocity()[2] << ","
        << m_origin->acceleration()[0] << "," << m_origin->acceleration()[1] << "," << m_origin->acceleration()[2] << ","
        << forceBody()[0] << "," << forceBody()[1] << "," << forceBody()[2] << ","
        << momentBody()[0] << "," << momentBody()[1] << "," << momentBody()[2];

    Logger::instance().log(LogType::Frame, msg.str());
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

frameRenderData Frame::packRenderData() const {
    frameRenderData data;
    data.transform      = m_transform;
    data.forceAtOrigin  = originNode().force();
    data.forceColor     = m_forceColor;
    data.name           = m_name;
    data.isCenterOfMass = m_isCenterOfMass;
    data.position       = originNode().position();
    data.velocity       = originNode().velocity();
    data.wind           = originNode().wind();
    return data;
}

