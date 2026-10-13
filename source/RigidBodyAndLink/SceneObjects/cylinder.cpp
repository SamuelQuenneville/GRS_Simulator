/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "cylinder.h"

Cylinder::Cylinder(const float radius, const float height, const uint32_t segments) {
    m_createMesh(radius, height, segments);
    m_createBuffers();
}

Cylinder::~Cylinder() {
    m_vertexBuffer.destroy();
}

void Cylinder::m_createMesh(const float radius, const float height, const uint32_t segments) {
    m_vertices.clear();

    for (uint32_t i = 0; i < segments; ++i) {
        const float theta0 = static_cast<float>(i) / segments * 2.0f * M_PIf;
        const float theta1 = static_cast<float>(i + 1) / segments * 2.0f * M_PIf;

        const float c0 = std::cos(theta0);
        const float s0 = std::sin(theta0);

        const float c1 = std::cos(theta1);
        const float s1 = std::sin(theta1);

        // positions (top/bottom ring)
        const grs::Vec3f p00(radius * c0, radius * s0, height);
        const grs::Vec3f p10(radius * c1, radius * s1, height);
        const grs::Vec3f p01(radius * c0, radius * s0, 0.0f);
        const grs::Vec3f p11(radius * c1, radius * s1, 0.0f);

        // normals (radial)
        const grs::Vec3f n0(c0, s0, 0.0f);
        const grs::Vec3f n1(c1, s1, 0.0f);

        // UVs
        float u0 = static_cast<float>(i) / segments;
        float u1 = static_cast<float>(i + 1) / segments;

        float vTop = 1.0f;
        float vBot = 0.0f;

        // Triangle 1
        m_vertices.push_back({.position = p00, .normal = n0, .texCoord = {u0, vTop}});
        m_vertices.push_back({.position = p01, .normal = n0, .texCoord = {u0, vBot}});
        m_vertices.push_back({.position = p10, .normal = n1, .texCoord = {u1, vTop}});

        // Triangle 2
        m_vertices.push_back({.position = p10, .normal = n1, .texCoord = {u1, vTop}});
        m_vertices.push_back({.position = p01, .normal = n0, .texCoord = {u0, vBot}});
        m_vertices.push_back({.position = p11, .normal = n1, .texCoord = {u1, vBot}});
    }

    m_vertexCount = static_cast<GLsizei>(m_vertices.size());
}

void Cylinder::m_createBuffers() {

    m_vertexArrayObject.create();
    m_vertexArrayObject.bind();

    // POSITION + NORMALS
    m_vertexBuffer.create();
    m_vertexBuffer.bind();
    m_vertexBuffer.copyDataIntoVBO(static_cast<GLsizei>(m_vertices.size() * sizeof(CylinderVertex)), m_vertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(VertexLayout::Position);
    glVertexAttribPointer(VertexLayout::Position, 3, GL_FLOAT, GL_FALSE, sizeof(CylinderVertex), nullptr);

    glEnableVertexAttribArray(VertexLayout::Normal);
    glVertexAttribPointer(VertexLayout::Normal, 3, GL_FLOAT, GL_FALSE, sizeof(CylinderVertex), reinterpret_cast<void *>(offsetof(CylinderVertex, normal)));

    m_vertexArrayObject.unbind();
}

void Cylinder::render(const Shader& shader, const grs::Matrix4d& modelMatrix, const grs::Matrix4d& worldToCameraMatrix) const {
    shader.bind();

    shader.setMatrix4x4Uniform("localToWorldMatrix", modelMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the vertex color
    shader.setVector4Uniform("globalVertexColor", Color::tetherColor().toVec4f(), false);

    // Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix of the model-view matrix)
    const grs::Matrix4d localToCameraMatrix = worldToCameraMatrix * modelMatrix;
    const grs::Matrix3d normalMatrix = localToCameraMatrix.upperLeft3x3().inverse().transpose();
    shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

    m_vertexArrayObject.bind();

    glDrawArrays(GL_TRIANGLES, 0, m_vertexCount);

    m_vertexArrayObject.unbind();

    shader.unbind();
}