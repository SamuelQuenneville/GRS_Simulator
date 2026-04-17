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

void PhysicsObject::renderVisualObject(Shader& shader, const grs::Matrix4d& worldToCameraMatrix, const bool drawFrames, const bool drawForces) {
    for (const auto& visualObject : m_visualObjects) {
        visualObject->render(shader, worldToCameraMatrix, drawFrames, drawForces);
    }
}

void PhysicsObject::setColor(const Color& color) {
    m_color = color;
}

const grs::TransformD& PhysicsObject::getTransform() const {
    return m_bodyRenderData.transform;
}

void PhysicsObject::setTransform(const grs::TransformD& transform) {
    m_bodyRenderData.transform = transform;
    updateTransform(1.0);
}

void PhysicsObject::setBodyRenderData(const bodyRenderData& renderData) {
    m_bodyRenderData = renderData;

    for (std::size_t i = 0; i < m_visualObjects.size(); ++i) {
        m_visualObjects[i]->setFrameRenderData(m_bodyRenderData.frames[i]);
    }
}

void PhysicsObject::setTetherRenderData(const tetherRenderData& renderData) {
    m_tetherRenderData = renderData;
}

void PhysicsObject::m_createVisualObject() {

    for (auto& f : m_bodyRenderData.frames) {
        auto visualObject = std::make_unique<FrameAxis>(f);
        m_visualObjects.push_back(std::move(visualObject));
    }
}

grs::Matrix4d PhysicsObject::m_calculateTransform(const double interpolationFactor, const grs::Matrix4d& scalingMatrix) {
    // Get the transform of the rigid body
    const grs::TransformD transform = m_bodyRenderData.transform;

    // Interpolate the transform between the previous one and the new one
    const grs::TransformD interpolatedTransform = grs::TransformD::interpolate(m_previousTransform, transform, interpolationFactor);
    m_previousTransform = transform;

    // Apply the scaling matrix
    return interpolatedTransform.toMatrix4() * scalingMatrix;
}
