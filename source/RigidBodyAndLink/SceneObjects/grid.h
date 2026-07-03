/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_GRID_H
#define GRS_SIMULATOR_GRID_H

#include "OpenglFramework/shader.h"
#include "OpenglFramework/vertexBufferObject.h"
#include "OpenglFramework/vertexArrayObject.h"
#include "Definitions/renderObjectDefinitions.h"
#include "Definitions/shaderLayout.h"

class Grid {
public:
    explicit Grid(float extent = 100.0f,float spacing = 5.0f);
    ~Grid();

    void render(const Shader& shader, const grs::Matrix4d& worldToCameraMatrix, const grs::Matrix4d& projectionMatrix) const;

private:

    void m_addLine(const grs::Vec3f& p1, const grs::Vec3f& p2, float width, uint32_t color);
    void m_createGrid(float extent, float spacing);
    void m_createBuffers();

    std::vector<Vertex> m_vertices;

    VertexBufferObject m_vertexBuffer{GL_ARRAY_BUFFER};
    VertexArrayObject m_vertexArrayObject;

    grs::Matrix4d m_transformMatrix = grs::Matrix4d::identity();
};

#endif //GRS_SIMULATOR_GRID_H
