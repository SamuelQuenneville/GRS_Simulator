/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "frameAxis.h"

FrameAxis::FrameAxis(RigidBody* body, const float length)
    : m_vertexBufferVertices(GL_ARRAY_BUFFER)
    , m_length(length)
{
    m_previousTransform = Transform::identity();
    m_body = body;

    m_createVertexBufferAndArrayObject();
}

FrameAxis::~FrameAxis() {
    destroy();

    m_body = nullptr;

    // Destroy the VBOs and VAO
    m_vertexBufferVertices.destroy();
}


void FrameAxis::render(Shader& shader, const Matrix4<double>& worldToCameraMatrix) {

    m_vertexArrayObject.bind();
    shader.bind();
    m_vertexBufferVertices.bind();

    // Set the model to camera matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", m_transformMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the vertex color
    shader.setIntUniform("isGlobalVertexColorEnabled", 0, false);

    // Get the location of shader attribute variables
    const GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    const GLint vertexColorLoc = shader.getAttribLocation("vertexColor");

    // Position attribute
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3<float>) + sizeof(uint32_t), nullptr);
    glEnableVertexAttribArray(vertexPositionLoc);

    // Color attribute
    glVertexAttribIPointer(vertexColorLoc, 3, GL_UNSIGNED_INT,  sizeof(Vector3<float>) + sizeof(uint32_t), reinterpret_cast<void *>(sizeof(Vector3<float>)));
    glEnableVertexAttribArray(vertexColorLoc);

    glDrawArrays(GL_LINES, 0, 6); // 3 arrows = 6 vertices

    m_vertexBufferVertices.unbind();
    m_vertexArrayObject.unbind();
    shader.unbind();
};

void FrameAxis::m_createVertexBufferAndArrayObject() {

    const auto x = FrameLine(Vector3<float>(0, 0, 0), Vector3<float>(m_length, 0, 0).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::RED));
    const auto y = FrameLine(Vector3<float>(0, 0, 0), Vector3<float>(0, m_length, 0).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::BLUE));
    const auto z = FrameLine(Vector3<float>(0, 0, 0), Vector3<float>(0, 0, m_length).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::GREEN));

    const std::vector<FrameLine> vertices = {x,y,z};

    m_vertexBufferVertices.create();
    m_vertexBufferVertices.bind();
    const auto sizeVertices = static_cast<GLsizei>(vertices.size() * sizeof(FrameLine));
    m_vertexBufferVertices.copyDataIntoVBO(sizeVertices, &vertices[0], GL_STATIC_DRAW);
    m_vertexBufferVertices.unbind();

    m_vertexArrayObject.create();
    m_vertexArrayObject.bind();
    m_vertexBufferVertices.bind();
    m_vertexArrayObject.unbind();
}

// Update the transform matrix of the object
void FrameAxis::updateTransform(const double interpolationFactor) {
    const auto scaling = Matrix4<double>::identity();
    m_transformMatrix = m_calculateTransform(interpolationFactor, scaling);
}