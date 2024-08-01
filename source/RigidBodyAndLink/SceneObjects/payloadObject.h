/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_PAYLOADOBJECT_H
#define GRS_SIMULATOR_PAYLOADOBJECT_H

#include "physicsObject.h"
#include "OpenglFramework/vertexBufferObject.h"
#include "OpenglFramework/vertexArrayObject.h"
#include "RigidBodyAndLink/payload.h"

class PayloadObject : public PhysicsObject {

public:
    PayloadObject(Payload* payload, const Vector3<float>& size, const std::string& executablePath);
    ~PayloadObject() override;

    void render(Shader& shader, const Matrix4<double>& worldToCameraMatrix) override;
    void updateTransform(double interpolationFactor) override;

private:
    float m_size[3] = {1,1,1};

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


#endif //GRS_SIMULATOR_PAYLOADOBJECT_H
