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

    // ground color
    constexpr auto color = static_cast<uint32_t>(Color::HexColor::GRASS);

    const float e = extent;

    // Triangle 1
    m_vertices.push_back({.position = grs::Vec3f(-e, y, -e), .color = color});
    m_vertices.push_back({.position = grs::Vec3f( e, y, -e), .color = color});
    m_vertices.push_back({.position = grs::Vec3f( e, y,  e), .color = color});

    // Triangle 2
    m_vertices.push_back({.position = grs::Vec3f(-e, y, -e), .color = color});
    m_vertices.push_back({.position = grs::Vec3f( e, y,  e), .color = color});
    m_vertices.push_back({.position = grs::Vec3f(-e, y,  e), .color = color});
}

void Ground::m_createBuffers() {

    m_vertexArrayObject.create();
    m_vertexArrayObject.bind();

    // POSITION + COLOR
    m_vertexBuffer.create();
    m_vertexBuffer.bind();
    m_vertexBuffer.copyDataIntoVBO(static_cast<GLsizei>(m_vertices.size() * sizeof(Vertex)), m_vertices.data(),GL_STATIC_DRAW);

    glEnableVertexAttribArray(VertexLayout::Position);
    glVertexAttribPointer(VertexLayout::Position, 3,GL_FLOAT,GL_FALSE,sizeof(Vertex),nullptr);

    glEnableVertexAttribArray(VertexLayout::Color);
    glVertexAttribIPointer(VertexLayout::Color,1,GL_UNSIGNED_INT,sizeof(Vertex),reinterpret_cast<void *>(offsetof(Vertex, color)));

    m_vertexArrayObject.unbind();
}

void Ground::render(const Shader& shader, const grs::Matrix4d& worldToCameraMatrix, const grs::Matrix4d& projectionMatrix) const {
    shader.bind();

    shader.setMatrix4x4Uniform("localToWorldMatrix", m_transformMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);
    shader.setMatrix4x4Uniform("projectionMatrix", projectionMatrix);

    shader.setIntUniform("isGlobalVertexColorEnabled",0, false);

    m_vertexArrayObject.bind();

    glDrawArrays(GL_TRIANGLES,0,static_cast<GLsizei>(m_vertices.size()));

    m_vertexArrayObject.unbind();

    shader.unbind();
}