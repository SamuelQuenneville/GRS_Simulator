/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "tetherLine.h"

TetherLine::TetherLine(Tether* tether)
    : m_vertexBufferVertices(GL_ARRAY_BUFFER)
{
    m_link = tether;
}

void TetherLine::render(Shader& shader, const grs::Matrix4d& worldToCameraMatrix) {

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    m_vertexArrayObject.bind();
    shader.bind();
    m_vertexBufferVertices.bind();

    // Set the model to camera matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", grs::Matrix4d::identity());
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the vertex color
    constexpr grs::Vec4f color(0.0, 0.0, 0.0, 1.0);
    shader.setIntUniform("isGlobalVertexColorEnabled", 1, false);
    shader.setVector4Uniform("globalVertexColor", color, false);

    // Get the location of shader attribute variables
    const GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");

    glEnableVertexAttribArray(vertexPositionLoc);
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    // Draw the lines
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_linePoints.size() * sizeof(grs::Vec3f)));
    glDisableVertexAttribArray(vertexPositionLoc);

    m_vertexBufferVertices.unbind();
    m_vertexArrayObject.unbind();
    shader.unbind();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void TetherLine::updatePosition() {
    m_point1 = grs::cast<float>(m_link->startPoint->position().toOpenglRef());
    m_point2 = grs::cast<float>(m_link->endPoint->position().toOpenglRef());

    m_linePoints.clear();
    m_linePoints.emplace_back(m_point1);
    m_linePoints.emplace_back(m_point2);

    m_createVertexBufferAndArrayObject();
}

void TetherLine::m_createVertexBufferAndArrayObject() {
    m_vertexBufferVertices.create();
    m_vertexBufferVertices.bind();
    const auto sizeVertices = static_cast<GLsizei>(m_linePoints.size() * sizeof(grs::Vec3f));
    m_vertexBufferVertices.copyDataIntoVBO(sizeVertices, &m_linePoints[0], GL_STATIC_DRAW);
    m_vertexBufferVertices.unbind();

    m_vertexArrayObject.create();
    m_vertexArrayObject.bind();
    m_vertexBufferVertices.bind();
    m_vertexArrayObject.unbind();
}

