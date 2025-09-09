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
    m_color = Color::white();
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

void PhysicsObject::renderVisualObject(Shader& shader, const Matrix4<double>& worldToCameraMatrix, const bool drawFrames, const bool drawForces) {
    for (const auto& visualObject : m_visualObjects) {
        visualObject->render(shader, worldToCameraMatrix, drawFrames, drawForces);
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
    updateTransform(1.0);
}

void PhysicsObject::m_createVisualObject() {
    if (m_body->frames().empty()) {
        auto visualObject = std::make_unique<FrameAxis>(m_body->bodyOriginFrame());
        m_visualObjects.push_back(std::move(visualObject));
    } else {

        for (auto& f : m_body->frames()) {
            auto visualObject = std::make_unique<FrameAxis>(f);
            m_visualObjects.push_back(std::move(visualObject));
        }
    }
}

Matrix4<double> PhysicsObject::m_calculateTransform(const double interpolationFactor, const Matrix4<double>& scalingMatrix) {
    // Get the transform of the rigid body
    const Transform transform = m_body->transform();

    // Interpolate the transform between the previous one and the new one
    const Transform interpolatedTransform = Transform::interpolateTransforms(m_previousTransform,transform, interpolationFactor);
    m_previousTransform = transform;

    // Compute the transform used for rendering the box
    double matrix[16];
    interpolatedTransform.getOpenGLMatrix(matrix);
    const Matrix4 newMatrix = {{matrix[0], matrix[4], matrix[8], matrix[12]},
                              {matrix[1], matrix[5], matrix[9], matrix[13]},
                              {matrix[2], matrix[6], matrix[10], matrix[14]},
                              {matrix[3], matrix[7], matrix[11], matrix[15]}};

    // Apply the scaling matrix to have the correct box dimensions
    return newMatrix * scalingMatrix;
}
