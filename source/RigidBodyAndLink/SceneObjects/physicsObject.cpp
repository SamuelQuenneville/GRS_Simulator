/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "physicsObject.h"

PhysicsObject::PhysicsObject()
    : Mesh()
{
    m_body = nullptr;
    m_link = nullptr;
    m_color = Color(1,1,1,1);
}

PhysicsObject::PhysicsObject(const std::string& meshFilePath)
    : PhysicsObject()
{
    // Load the mesh from a file
    MeshReader::loadMesh(meshFilePath, *this);

    // If the mesh file do not have normals
    if (m_normals.empty()) {
        calculateNormals();
    }
}

void PhysicsObject::setColor(const Color& color) {
    m_color = color;
}

const Transform& PhysicsObject::getTransform() const {
    return m_body->transform();
}

void PhysicsObject::setTransform(const Transform& transform) {
    m_body->setTransform(transform);
    updateTransform(1.0f);
}

Matrix4<double> PhysicsObject::m_calculateTransform(double interpolationFactor, const Matrix4<double>& scalingMatrix) {
    // Get the transform of the rigid body
    Transform transform = m_body->transform();

    // Interpolate the transform between the previous one and the new one
    Transform interpolatedTransform = Transform::interpolateTransforms(m_previousTransform,transform, interpolationFactor);
    m_previousTransform = transform;

    // Compute the transform used for rendering the box
    double matrix[16];
    interpolatedTransform.getOpenGLMatrix(matrix);
    Matrix4<double> newMatrix = {{matrix[0], matrix[4], matrix[8], matrix[12]},
                                 {matrix[1], matrix[5], matrix[9], matrix[13]},
                                 {matrix[2], matrix[6], matrix[10], matrix[14]},
                                 {matrix[3], matrix[7], matrix[11], matrix[15]}};

    // Apply the scaling matrix to have the correct box dimensions
    return newMatrix * scalingMatrix;
}
