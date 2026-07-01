/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_GROUND_H
#define GRS_SIMULATOR_GROUND_H

#include "OpenglFramework/shader.h"
#include "OpenglFramework/vertexBufferObject.h"
#include "OpenglFramework/vertexArrayObject.h"
#include "Definitions/renderObjectDefinitions.h"

class Ground {
public:
    explicit Ground(float extent = 100.0f, float y = 0.0f);
    ~Ground();

    void render(const Shader& shader, const grs::Matrix4d& worldToCameraMatrix, const grs::Matrix4d& projectionMatrix) const;

private:

    void m_createGeometry(float extent, float y);
    void m_createBuffers();

    std::vector<Vertex> m_vertices;

    VertexBufferObject m_vertexBuffer{GL_ARRAY_BUFFER};
    VertexArrayObject m_vertexArrayObject;

    grs::Matrix4d m_transformMatrix = grs::Matrix4d::identity();
};


#endif //GRS_SIMULATOR_GROUND_H
