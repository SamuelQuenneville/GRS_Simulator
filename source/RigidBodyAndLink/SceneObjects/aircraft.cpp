/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "aircraft.h"

// Constructor
Aircraft::Aircraft(const bodyRenderData& aircraftRenderData, const grs::Vec3d& scaling, const std::string& executablePath)
    : PhysicsObject(executablePath + R"(/Assets/Meshes/aircraft.obj)")
{
    m_scalingMatrix = grs::Matrix4d::diagonal({scaling[0], scaling[1], scaling[2], 1.0});

    m_previousTransform = grs::TransformD::identity();

    m_bodyRenderData = aircraftRenderData;

    // Create Frames
    m_createVisualObject();

    // Create the VBOs and VAO
    createVertexBufferAndArrayObject();

    setTransformMatrix(getTransformMatrix() * m_scalingMatrix);
}

void Aircraft::setDisplayScale(const double scale) {
    m_scalingMatrix = grs::Matrix4d::diagonal({scale, scale, scale, 1.0});
    updateTransform(1.0);
}

// Update the transform matrix of the object
void Aircraft::updateTransform(const double interpolationFactor) {
    setTransformMatrix(m_calculateTransform(interpolationFactor, m_scalingMatrix));

    for (const auto& visualObject : m_visualObjects) {
        visualObject->updateTransform(interpolationFactor);
        visualObject->updateForceVector();
    }
}
