/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "launcher.h"

// Constructor
Launcher::Launcher(const grs::Vec3d& scaling, const std::string& executablePath)
    : PhysicsObject(executablePath + R"(/Assets/Meshes/launcher.obj)")
{
    m_scalingMatrix = grs::Matrix4d::diagonal({scaling[0], scaling[1], scaling[2], 1.0});

    m_previousTransform = grs::TransformD::identity();

    // Create the VBOs and VAO
    createVertexBufferAndArrayObject();

    setTransformMatrix(getTransformMatrix() * m_scalingMatrix);
}

// Update the transform matrix of the object
void Launcher::updateTransform(const double interpolationFactor) {
    setTransformMatrix(m_calculateTransform(interpolationFactor, m_scalingMatrix));
}