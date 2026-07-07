/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "cube.h"

Cube::Cube(const grs::Vec3f& size, const std::string& executablePath)
    : PhysicsObject(executablePath + R"(/Assets/Meshes/cube.obj)")
{
    // Initialize the size of the box
    m_size[0] = size[0];
    m_size[1] = size[1];
    m_size[2] = size[2];

    // Compute the scaling matrix
    m_scalingMatrix = grs::Matrix4d::diagonal({m_size[0], m_size[1], m_size[2], 1.0});

    m_previousTransform = grs::TransformD::identity();

    // Create the VBOs and VAO
    createVertexBufferAndArrayObject();

    setTransformMatrix(getTransformMatrix() * m_scalingMatrix);
}

void Cube::updateTransform(const double interpolationFactor) {
    setTransformMatrix(m_calculateTransform(interpolationFactor, m_scalingMatrix));
}