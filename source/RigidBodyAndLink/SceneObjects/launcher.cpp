/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "launcher.h"

// Constructor
Launcher::Launcher(const Vector3<double>& scaling, const std::string& executablePath)
        : PhysicsObject(executablePath + R"(/source/Meshes/launcher.obj)")
        , m_vertexBufferVertices(GL_ARRAY_BUFFER)
        , m_vertexBufferNormals(GL_ARRAY_BUFFER)
        , m_vertexBufferTextureCoords(GL_ARRAY_BUFFER)
        , m_vertexBufferIndices(GL_ELEMENT_ARRAY_BUFFER)
{
    m_scalingMatrix = Matrix4<double>{{scaling.x(),           0,           0, 0},
                                      {           0, scaling.y(),           0, 0},
                                      {           0,           0, scaling.z(), 0},
                                      {           0,           0,           0, 1}};

    m_previousTransform = Transform::identity();

    m_body = new RigidBody();;

    // Create the VBOs and VAO
    m_createVertexBufferAndArrayObject();

    m_transformMatrix = m_transformMatrix * m_scalingMatrix;
}

// Destructor
Launcher::~Launcher() {
    destroy();

    // Destroy the VBOs and VAO
    m_vertexBufferIndices.destroy();
    m_vertexBufferVertices.destroy();
    m_vertexBufferNormals.destroy();
    m_vertexBufferTextureCoords.destroy();
    m_vertexArrayObject.destroy();
}

// Render the sphere at the correct position and with the correct orientation
void Launcher::render(Shader& shader, const Matrix4<double>& worldToCameraMatrix) {

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
    const Vector4 color(m_color.r, m_color.g, m_color.b, m_color.a);
    shader.setVector4Uniform("globalVertexColor", color, false);

    // Bind the VAO
    m_vertexArrayObject.bind();

    m_vertexBufferVertices.bind();

    // Get the location of shader attribute variables
    const GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    const GLint vertexNormalLoc = shader.getAttribLocation("vertexNormal", false);

    glEnableVertexAttribArray(vertexPositionLoc);
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    m_vertexBufferNormals.bind();

    if (vertexNormalLoc != -1) glVertexAttribPointer(vertexNormalLoc, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    if (vertexNormalLoc != -1) glEnableVertexAttribArray(vertexNormalLoc);

    // For each part of the mesh
    for (int i=0; i<getNumbersParts(); i++) {
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(getNumbersFaces(i) * 3), GL_UNSIGNED_INT, nullptr);
    }

    glDisableVertexAttribArray(vertexPositionLoc);
    if (vertexNormalLoc != -1) glDisableVertexAttribArray(vertexNormalLoc);

    m_vertexBufferNormals.unbind();
    m_vertexBufferVertices.unbind();

    // Unbind the VAO
    m_vertexArrayObject.unbind();

    // Unbind the shader
    shader.unbind();
}

// Create the Vertex Buffer Objects used to render with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void Launcher::m_createVertexBufferAndArrayObject() {

    // Create the VBO for the vertices data
    m_vertexBufferVertices.create();
    m_vertexBufferVertices.bind();
    const auto sizeVertices = static_cast<GLsizei>(m_vertices.size() * sizeof(Vector3<float>));
    m_vertexBufferVertices.copyDataIntoVBO(sizeVertices, getVerticesPointer(), GL_STATIC_DRAW);
    m_vertexBufferVertices.unbind();

    // Create the VBO for the normals data
    m_vertexBufferNormals.create();
    m_vertexBufferNormals.bind();
    const auto sizeNormals = static_cast<GLsizei>(m_normals.size() * sizeof(Vector3<float>));
    m_vertexBufferNormals.copyDataIntoVBO(sizeNormals, getNormalsPointer(), GL_STATIC_DRAW);
    m_vertexBufferNormals.unbind();

    if (hasTexture()) {
        // Create the VBO for the texture co data
        m_vertexBufferTextureCoords.create();
        m_vertexBufferTextureCoords.bind();
        const auto sizeTextureCoords = static_cast<GLsizei>(m_uvs.size() * sizeof(Vector2<float>));
        m_vertexBufferTextureCoords.copyDataIntoVBO(sizeTextureCoords, getUVTextureCoordinatesPointer(), GL_STATIC_DRAW);
        m_vertexBufferTextureCoords.unbind();
    }

    // Create th VBO for the indices data
    m_vertexBufferIndices.create();
    m_vertexBufferIndices.bind();
    const auto sizeIndices = static_cast<GLsizei>(m_indices[0].size() * sizeof(unsigned int));
    m_vertexBufferIndices.copyDataIntoVBO(sizeIndices, getIndicesPointer(), GL_STATIC_DRAW);
    m_vertexBufferIndices.unbind();

    // Create the VAO for both VBOs
    m_vertexArrayObject.create();
    m_vertexArrayObject.bind();

    // Bind the VBO of vertices
    m_vertexBufferVertices.bind();
    m_vertexBufferNormals.bind();

    if (hasTexture()) {
        m_vertexBufferTextureCoords.bind();
    }

    m_vertexBufferIndices.bind();
    m_vertexArrayObject.unbind();
}

// Update the transform matrix of the object
void Launcher::updateTransform(const double interpolationFactor) {
    m_transformMatrix = m_calculateTransform(interpolationFactor, m_scalingMatrix);
}