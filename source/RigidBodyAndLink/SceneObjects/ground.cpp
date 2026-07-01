/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "ground.h"

Ground::Ground(const float extent, const float y) {
    m_createGeometry(extent, y);
    m_createBuffers();
}

Ground::~Ground() {
    m_vertexBuffer.destroy();
    m_vertexArrayObject.destroy();
}

void Ground::m_createGeometry(const float extent, const float y) {
    m_vertices.clear();

    // dark ground color (RGBA packed)
    constexpr uint32_t color = 0x303030;

    const float e = extent;

    // Triangle 1
    m_vertices.push_back({grs::Vec3f(-e, y, -e), color});
    m_vertices.push_back({grs::Vec3f( e, y, -e), color});
    m_vertices.push_back({grs::Vec3f( e, y,  e), color});

    // Triangle 2
    m_vertices.push_back({grs::Vec3f(-e, y, -e), color});
    m_vertices.push_back({grs::Vec3f( e, y,  e), color});
    m_vertices.push_back({grs::Vec3f(-e, y,  e), color});
}

void Ground::m_createBuffers() {
    m_vertexBuffer.create();
    m_vertexBuffer.bind();

    m_vertexBuffer.copyDataIntoVBO(static_cast<GLsizei>(m_vertices.size() * sizeof(Vertex)), m_vertices.data(),GL_STATIC_DRAW);

    m_vertexBuffer.unbind();

    m_vertexArrayObject.create();
    m_vertexArrayObject.bind();

    m_vertexBuffer.bind();

    // position
    glVertexAttribPointer(0, 3,GL_FLOAT,GL_FALSE,sizeof(Vertex),nullptr);
    glEnableVertexAttribArray(0);

    // color (uint32)
    glVertexAttribIPointer(1,1,GL_UNSIGNED_INT,sizeof(Vertex),reinterpret_cast<void *>(offsetof(Vertex, color)));
    glEnableVertexAttribArray(1);

    m_vertexBuffer.unbind();
    m_vertexArrayObject.unbind();
}

void Ground::render(const Shader& shader, const grs::Matrix4d& worldToCameraMatrix, const grs::Matrix4d& projectionMatrix) const {
    shader.bind();

    shader.setMatrix4x4Uniform("localToWorldMatrix", m_transformMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);
    shader.setMatrix4x4Uniform("projectionMatrix", projectionMatrix);

    shader.setIntUniform("isGlobalVertexColorEnabled",0, false);

    m_vertexArrayObject.bind();

    m_vertexBuffer.bind();

    // (optional but consistent with your engine style)
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),nullptr);
    glVertexAttribIPointer(1,1,GL_UNSIGNED_INT, sizeof(Vertex), reinterpret_cast<void *>(offsetof(Vertex, color)));

    glDrawArrays( GL_TRIANGLES,0,static_cast<GLsizei>(m_vertices.size()));

    m_vertexBuffer.unbind();
    m_vertexArrayObject.unbind();

    shader.unbind();
}