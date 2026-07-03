/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_AIRCRAFT_H
#define GRS_SIMULATOR_AIRCRAFT_H

#include "physicsObject.h"
#include "OpenglFramework/vertexBufferObject.h"
#include "OpenglFramework/vertexArrayObject.h"
#include "RigidBodyAndLink/fixedWing.h"
#include "Definitions/shaderLayout.h"

class Aircraft final : public PhysicsObject {

public :
    Aircraft(const bodyRenderData &aircraftRenderData, const grs::Vec3d& scaling, const std::string& executablePath);
    ~Aircraft() override;

    void render(Shader& shader, const grs::Matrix4d& worldToCameraMatrix) override;
    void updateTransform(double interpolationFactor) override;

private :
    grs::Matrix4d m_scalingMatrix{};

    // Vertex Buffer Object for the vertices data
    VertexBufferObject m_vertexBufferVertices;

    // Vertex Buffer Object for the normals data
    VertexBufferObject m_vertexBufferNormals;

    // Vertex Buffer Object for the texture coords
    VertexBufferObject m_vertexBufferTextureCoords;

    // Vertex Buffer Object for the indices
    VertexBufferObject m_vertexBufferIndices;

    // Vertex Array Object for the vertex data
    VertexArrayObject m_vertexArrayObject;

    void m_createVertexBufferAndArrayObject();
};


#endif //GRS_SIMULATOR_AIRCRAFT_H
