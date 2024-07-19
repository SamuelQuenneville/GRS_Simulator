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

class Aircraft : public PhysicsObject {

private :

    /// Scaling matrix
    Matrix4<double> mScalingMatrix;

    /// Vertex Buffer Object for the vertices data
    VertexBufferObject mVBOVertices;

    /// Vertex Buffer Object for the normals data
    VertexBufferObject mVBONormals;

    /// Vertex Buffer Object for the texture coords
    VertexBufferObject mVBOTextureCoords;

    /// Vertex Buffer Object for the indices
    VertexBufferObject mVBOIndices;

    /// Vertex Array Object for the vertex data
    VertexArrayObject mVAO;

    // Create the Vertex Buffer Objects used to render with OpenGL.
    void createVBOAndVAO();

public :

    Aircraft(const Vector3<double>& scaling = Vector3<double>(1, 1, 1));
    ~Aircraft() override;

    /// Render the mesh at the correct position and with the correct orientation
    void render(Shader& shader, const Matrix4<double>& worldToCameraMatrix) override;

    /// Update the transform matrix of the object
    void updateTransform(double interpolationFactor) override;
};


#endif //GRS_SIMULATOR_AIRCRAFT_H
