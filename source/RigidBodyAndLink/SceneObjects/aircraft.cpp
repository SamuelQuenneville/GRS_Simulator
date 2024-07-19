/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "aircraft.h"

// Constructor
Aircraft::Aircraft(const Vector3<double>& scaling)
        : PhysicsObject("C:/Users/ques2902/Desktop/Meshes/aircraft.obj")
        , mVBOVertices(GL_ARRAY_BUFFER)
        , mVBONormals(GL_ARRAY_BUFFER)
        , mVBOTextureCoords(GL_ARRAY_BUFFER)
        , mVBOIndices(GL_ELEMENT_ARRAY_BUFFER)
{


    mScalingMatrix = Matrix4<double>{{scaling.x(),           0,           0, 0},
                                     {          0, scaling.y(),           0, 0},
                                     {          0,           0, scaling.z(), 0},
                                     {          0,           0,           0, 1}};

    m_previousTransform = Transform::identity();

    m_body = new RigidBody();

    // Create the VBOs and VAO
    createVBOAndVAO();

    m_transformMatrix = m_transformMatrix * mScalingMatrix;
}

// Destructor
Aircraft::~Aircraft() {

    // Destroy the mesh
    destroy();

    // Destroy the VBOs and VAO
    mVBOIndices.destroy();
    mVBOVertices.destroy();
    mVBONormals.destroy();
    mVBOTextureCoords.destroy();
    mVAO.destroy();
}

// Render the sphere at the correct position and with the correct orientation
void Aircraft::render(Shader& shader, const Matrix4<double>& worldToCameraMatrix) {

    // Bind the shader
    shader.bind();

    // Set the model to camera matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", m_transformMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix of the model-view matrix)
    const Matrix4<double> localToCameraMatrix = worldToCameraMatrix * m_transformMatrix;
    const Matrix3<double> normalMatrix = localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().transpose();
    shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

    // Set the vertex color
    auto* rigidBody = dynamic_cast<RigidBody*>(m_body);
    Vector4<double> color(m_color.r, m_color.g, m_color.b, m_color.a);
    shader.setVector4Uniform("globalVertexColor", color, false);

    // Bind the VAO
    mVAO.bind();

    mVBOVertices.bind();

    // Get the location of shader attribute variables
    GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    GLint vertexNormalLoc = shader.getAttribLocation("vertexNormal", false);

    glEnableVertexAttribArray(vertexPositionLoc);
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)nullptr);

    mVBONormals.bind();

    if (vertexNormalLoc != -1) glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)nullptr);
    if (vertexNormalLoc != -1) glEnableVertexAttribArray(vertexNormalLoc);

    // For each part of the mesh
    for (unsigned int i=0; i<getNumbersParts(); i++) {
        glDrawElements(GL_TRIANGLES, getNumbersFaces(i) * 3, GL_UNSIGNED_INT, (char*)nullptr);
    }

    glDisableVertexAttribArray(vertexPositionLoc);
    if (vertexNormalLoc != -1) glDisableVertexAttribArray(vertexNormalLoc);

    mVBONormals.unbind();
    mVBOVertices.unbind();

    // Unbind the VAO
    mVAO.unbind();

    // Unbind the shader
    shader.unbind();
}

// Create the Vertex Buffer Objects used to render with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void Aircraft::createVBOAndVAO() {

    // Create the VBO for the vertices data
    mVBOVertices.create();
    mVBOVertices.bind();
    size_t sizeVertices = m_vertices.size() * sizeof(Vector3<float>);
    mVBOVertices.copyDataIntoVBO(sizeVertices, getVerticesPointer(), GL_STATIC_DRAW);
    mVBOVertices.unbind();

    // Create the VBO for the normals data
    mVBONormals.create();
    mVBONormals.bind();
    size_t sizeNormals = m_normals.size() * sizeof(Vector3<float>);
    mVBONormals.copyDataIntoVBO(sizeNormals, getNormalsPointer(), GL_STATIC_DRAW);
    mVBONormals.unbind();

    if (hasTexture()) {
        // Create the VBO for the texture co data
        mVBOTextureCoords.create();
        mVBOTextureCoords.bind();
        size_t sizeTextureCoords = m_uvs.size() * sizeof(Vector2<float>);
        mVBOTextureCoords.copyDataIntoVBO(sizeTextureCoords, getUVTextureCoordinatesPointer(), GL_STATIC_DRAW);
        mVBOTextureCoords.unbind();
    }

    // Create th VBO for the indices data
    mVBOIndices.create();
    mVBOIndices.bind();
    size_t sizeIndices = m_indices[0].size() * sizeof(unsigned int);
    mVBOIndices.copyDataIntoVBO(sizeIndices, getIndicesPointer(), GL_STATIC_DRAW);
    mVBOIndices.unbind();

    // Create the VAO for both VBOs
    mVAO.create();
    mVAO.bind();

    // Bind the VBO of vertices
    mVBOVertices.bind();

    // Bind the VBO of normals
    mVBONormals.bind();

    if (hasTexture()) {
        // Bind the VBO of texture coords
        mVBOTextureCoords.bind();
    }

    // Bind the VBO of indices
    mVBOIndices.bind();

    // Unbind the VAO
    mVAO.unbind();
}

// Update the transform matrix of the object
void Aircraft::updateTransform(double interpolationFactor) {
    m_transformMatrix = m_calculateTransform(interpolationFactor, mScalingMatrix);
}
