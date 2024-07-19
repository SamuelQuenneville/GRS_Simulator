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

class Aircraft : public PhysicsObject {

public :
    Aircraft(FixedWing* aircraft, const Vector3<double>& scaling = Vector3<double>(1, 1, 1));
    ~Aircraft() override;

    void render(Shader& shader, const Matrix4<double>& worldToCameraMatrix) override;
    void updateTransform(double interpolationFactor) override;

private :
    Matrix4<double> m_scalingMatrix;

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
