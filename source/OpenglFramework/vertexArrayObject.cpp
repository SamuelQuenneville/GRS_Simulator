/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "vertexArrayObject.h"

VertexArrayObject::VertexArrayObject()
    : m_vertexArrayId(0)
{

}

VertexArrayObject::~VertexArrayObject() {
    destroy();
}

bool VertexArrayObject::create() {
    // Destroy the current VAO
    destroy();

    // Generate a new VAO
    glGenVertexArrays(1, &m_vertexArrayId);
    assert(m_vertexArrayId != 0);

    return true;
}

void VertexArrayObject::bind() const {
    assert(m_vertexArrayId != 0);
    // Bind the VAO
    glBindVertexArray(m_vertexArrayId);
}

void VertexArrayObject::unbind() const {
    assert(m_vertexArrayId != 0);
    // Unbind the VAO
    glBindVertexArray(0);
}

void VertexArrayObject::destroy() {
    if (m_vertexArrayId != 0) {
        glDeleteVertexArrays(1, &m_vertexArrayId);
        m_vertexArrayId = 0;
    }
}
