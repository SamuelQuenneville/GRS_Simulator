/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_CYLINDER_H
#define GRS_SIMULATOR_CYLINDER_H

#include <vector>

#include "OpenglFramework/shader.h"
#include "OpenglFramework/vertexArrayObject.h"
#include "OpenglFramework/vertexBufferObject.h"
#include "Mathematics/color.h"
#include "Definitions/shaderLayout.h"

struct CylinderVertex {
    grs::Vec3f position;
    grs::Vec3f normal;
    grs::Vec2f texCoord;
};

class Cylinder {
public:
    explicit Cylinder(float radius = 1.0f, float height = 1.0f, uint32_t segments = 12);
    ~Cylinder();

    void render(const Shader& shader, const grs::Matrix4d& modelMatrix, const grs::Matrix4d& worldToCameraMatrix) const;

private:
    void m_createMesh(float radius, float height, uint32_t segments);
    void m_createBuffers();

    std::vector<CylinderVertex> m_vertices;

    VertexBufferObject m_vertexBuffer{GL_ARRAY_BUFFER};
    VertexArrayObject  m_vertexArrayObject;

    GLsizei m_vertexCount = 0;
};

#endif //GRS_SIMULATOR_CYLINDER_H
