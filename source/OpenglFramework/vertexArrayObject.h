/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_VERTEXARRAYOBJECT_H
#define GRS_SIMULATOR_VERTEXARRAYOBJECT_H

#include <nanogui/opengl.h>

class VertexArrayObject {

public:
    VertexArrayObject();
    ~VertexArrayObject();

    bool create();

    void bind() const;
    void unbind() const;

    void destroy();

private:
    GLuint m_vertexArrayId;
};


#endif //GRS_SIMULATOR_VERTEXARRAYOBJECT_H
