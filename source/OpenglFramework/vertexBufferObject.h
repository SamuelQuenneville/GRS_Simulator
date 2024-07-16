/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_VERTEXBUFFEROBJECT_H
#define GRS_SIMULATOR_VERTEXBUFFEROBJECT_H

#include <nanogui/opengl.h>

class VertexBufferObject {

public:
    explicit VertexBufferObject(GLenum targetData);
    ~VertexBufferObject();

    bool create();

    void copyDataIntoVBO(GLsizei size, const void* data, GLenum usage) const;

    void bind() const;
    void unbind() const;

    void destroy();

private:
    GLuint m_vertexBufferId;
    GLuint m_targetData;
};


#endif //GRS_SIMULATOR_VERTEXBUFFEROBJECT_H
