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
    , m_vertexBufferVertices(GL_ARRAY_BUFFER)
    , m_vertexBufferNormals(GL_ARRAY_BUFFER)
    , m_vertexBufferTextureCoords(GL_ARRAY_BUFFER)
    , m_vertexBufferIndices(GL_ELEMENT_ARRAY_BUFFER)
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

PhysicsObject::~PhysicsObject() {
    m_vertexBufferIndices.destroy();
    m_vertexBufferVertices.destroy();
    m_vertexBufferNormals.destroy();
    m_vertexBufferTextureCoords.destroy();
    m_vertexArrayObject.destroy();
}

// Render the sphere at the correct position and with the correct orientation
void PhysicsObject::render(Shader& shader, const grs::Matrix4d& worldToCameraMatrix) {

    shader.bind();

    // Set the model to camera matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", getTransformMatrix());
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix of the model-view matrix)
    const grs::Matrix4d localToCameraMatrix = worldToCameraMatrix * getTransformMatrix();
    const grs::Matrix3d normalMatrix = localToCameraMatrix.upperLeft3x3().inverse().transpose();
    shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

    // Set the vertex color
    shader.setVector4Uniform("globalVertexColor", m_color.toVec4f(), false);

    m_vertexArrayObject.bind();

    // For each part of the mesh
    for (int i=0; i<getNumbersParts(); i++) {
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(getNumbersFaces(i) * 3), GL_UNSIGNED_INT, nullptr);
    }

    m_vertexArrayObject.unbind();

    shader.unbind();
}

void PhysicsObject::renderWithModel(Shader& shader, const grs::Matrix4d& localToWorldMatrix, const grs::Matrix4d& worldToCameraMatrix) {
    shader.bind();

    shader.setMatrix4x4Uniform("localToWorldMatrix", localToWorldMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    const grs::Matrix4d localToCameraMatrix = worldToCameraMatrix * localToWorldMatrix;
    const grs::Matrix3d normalMatrix = localToCameraMatrix.upperLeft3x3().inverse().transpose();
    shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);
    shader.setVector4Uniform("globalVertexColor", m_color.toVec4f(), false);

    m_vertexArrayObject.bind();
    for (int i=0; i<getNumbersParts(); i++) {
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(getNumbersFaces(i) * 3), GL_UNSIGNED_INT, nullptr);
    }
    m_vertexArrayObject.unbind();

    shader.unbind();
}

void PhysicsObject::renderVisualObject(Shader& shader, const grs::Matrix4d& worldToCameraMatrix, const bool drawFrames, const bool drawForces) {
    for (const auto& visualObject : m_visualObjects) {
        visualObject->render(shader, worldToCameraMatrix, drawFrames, drawForces);
    }
}

// Create the Vertex Buffer Objects used to render with OpenGL.
// We create two VBOs (one for vertices and one for indices)
void PhysicsObject::createVertexBufferAndArrayObject() {

    m_vertexArrayObject.create();
    m_vertexArrayObject.bind();

    // POSITION
    m_vertexBufferVertices.create();
    m_vertexBufferVertices.bind();
    m_vertexBufferVertices.copyDataIntoVBO(static_cast<GLsizei>(m_vertices.size() * sizeof(grs::Vec3f)), getVerticesPointer(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(VertexLayout::Position);
    glVertexAttribPointer(VertexLayout::Position, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    // NORMALS
    m_vertexBufferNormals.create();
    m_vertexBufferNormals.bind();
    m_vertexBufferNormals.copyDataIntoVBO(static_cast<GLsizei>(m_normals.size() * sizeof(grs::Vec3f)), getNormalsPointer(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(VertexLayout::Normal);
    glVertexAttribPointer(VertexLayout::Normal, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    // UVs
    if (hasTexture()) {
        m_vertexBufferTextureCoords.create();
        m_vertexBufferTextureCoords.bind();
        m_vertexBufferTextureCoords.copyDataIntoVBO(static_cast<GLsizei>(m_uvs.size() * sizeof(grs::Vec2f)), getUVTextureCoordinatesPointer(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(VertexLayout::TexCoord);
        glVertexAttribPointer(VertexLayout::TexCoord,2, GL_FLOAT,GL_FALSE,0, nullptr);
    }

    // INDICES
    m_vertexBufferIndices.create();
    m_vertexBufferIndices.bind();
    m_vertexBufferIndices.copyDataIntoVBO(static_cast<GLsizei>(m_indices[0].size() * sizeof(unsigned int)), getIndicesPointer(), GL_STATIC_DRAW);

    m_vertexArrayObject.unbind();
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
