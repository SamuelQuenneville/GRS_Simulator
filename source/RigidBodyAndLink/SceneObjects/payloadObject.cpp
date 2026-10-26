/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "payloadObject.h"

PayloadObject::PayloadObject(const bodyRenderData& payloadRenderData, const grs::Vec3f& size, const std::string& executablePath)
    : PhysicsObject(executablePath + R"(/Assets/Meshes/cube.obj)")
    , m_vertexBufferVertices(GL_ARRAY_BUFFER)
    , m_vertexBufferNormals(GL_ARRAY_BUFFER)
    , m_vertexBufferTextureCoords(GL_ARRAY_BUFFER)
    , m_vertexBufferIndices(GL_ELEMENT_ARRAY_BUFFER)
{
    // Initialize the size of the box
    m_size[0] = size[0];
    m_size[1] = size[1];
    m_size[2] = size[2];

    // Compute the scaling matrix
    m_scalingMatrix = grs::Matrix4d::diagonal({m_size[0], m_size[1], m_size[2], 1.0});

    m_previousTransform = grs::TransformD::identity();

    m_bodyRenderData = payloadRenderData;

    // Create Frames
    m_createVisualObject();

    // Create the VBOs and VAO
    m_createVertexBufferAndArrayObject();

    setTransformMatrix(getTransformMatrix() * m_scalingMatrix);
}

PayloadObject::~PayloadObject() {
    m_vertexBufferVertices.destroy();
    m_vertexBufferNormals.destroy();
    m_vertexArrayObject.destroy();
}

void PayloadObject::renderWithModel(Shader& shader, const grs::Matrix4d& localToWorldMatrix, const grs::Matrix4d& worldToCameraMatrix) {
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

void PayloadObject::render(Shader& shader, const grs::Matrix4d& worldToCameraMatrix) {

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

void PayloadObject::updateTransform(const double interpolationFactor) {
    setTransformMatrix(m_calculateTransform(interpolationFactor, m_scalingMatrix));

    for (const auto& visualObject : m_visualObjects) {
        visualObject->updateTransform(interpolationFactor);
        visualObject->updateForceVector();
    }
}

void PayloadObject::m_createVertexBufferAndArrayObject() {

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
