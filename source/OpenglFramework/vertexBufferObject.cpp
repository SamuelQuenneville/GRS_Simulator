/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "vertexBufferObject.h"

VertexBufferObject::VertexBufferObject(const GLenum targetData)
    : m_vertexBufferId(0)
    , m_targetData(targetData)
{

}

VertexBufferObject::~VertexBufferObject() {
    destroy();
}

bool VertexBufferObject::create() {
    // Destroy the current VBO
    destroy();

    // Generate a new VBO
    glGenBuffers(1, &m_vertexBufferId);
    assert(m_vertexBufferId != 0);

    return true;
}

void VertexBufferObject::copyDataIntoVBO(const GLsizei size, const void* data, const GLenum usage) const {
    glBufferData(m_targetData, size, data, usage);
}

void VertexBufferObject::bind() const {
    assert(m_vertexBufferId != 0);
    // Bind the VBO
    glBindBuffer(m_targetData, m_vertexBufferId);
}

void VertexBufferObject::unbind() const {
    assert(m_vertexBufferId != 0);
    // Unbind the VBO
    glBindBuffer(m_targetData, 0);
}

void VertexBufferObject::destroy() {
    if (m_vertexBufferId != 0) {
        glDeleteFramebuffers(1, &m_vertexBufferId);
        m_vertexBufferId = 0;
    }
}
