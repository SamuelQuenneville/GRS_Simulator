/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "frameAxis.h"

FrameAxis::FrameAxis(Frame* frame, const float length)
    : m_frame(frame)
    , m_vertexBufferVertices(GL_ARRAY_BUFFER)
    , m_vertexBufferForce(GL_ARRAY_BUFFER)
    , m_length(length)
{
    m_previousTransform = Transform::identity();

    m_createVertexBufferAndArrayObject();
    m_createForceBufferAndArrayObject();
}

FrameAxis::~FrameAxis() {
    // Destroy the VBOs and VAO
    m_vertexBufferVertices.destroy();
    m_vertexBufferForce.destroy();
}


void FrameAxis::render(const Shader& shader, const Matrix4<double>& worldToCameraMatrix, const bool drawFrames, const bool drawForces) const {

    shader.bind();

    // Set the model to camera matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", m_transformMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the vertex color
    shader.setIntUniform("isGlobalVertexColorEnabled", 0, false);

    // Get the location of shader attribute variables
    const GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    const GLint vertexColorLoc = shader.getAttribLocation("vertexColor");


    if (drawFrames) {
        m_vertexArrayObject.bind();
        m_vertexBufferVertices.bind();

        // Position attribute
        glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3<float>) + sizeof(uint32_t), nullptr);
        glEnableVertexAttribArray(vertexPositionLoc);

        // Color attribute
        glVertexAttribIPointer(vertexColorLoc, 3, GL_UNSIGNED_INT,  sizeof(Vector3<float>) + sizeof(uint32_t), reinterpret_cast<void *>(sizeof(Vector3<float>)));
        glEnableVertexAttribArray(vertexColorLoc);

        glDrawArrays(GL_LINES, 0, 6); // 3 arrows = 6 vertices

        m_vertexBufferVertices.unbind();
        m_vertexArrayObject.unbind();
    }


    if (drawForces) {
        m_vertexArrayObjectForce.bind();
        m_vertexBufferForce.bind();

        // Position attribute
        glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3<float>) + sizeof(uint32_t), nullptr);
        glEnableVertexAttribArray(vertexPositionLoc);

        // Color attribute
        glVertexAttribIPointer(vertexColorLoc, 3, GL_UNSIGNED_INT, sizeof(Vector3<float>) + sizeof(uint32_t), reinterpret_cast<void*>(sizeof(Vector3<float>)));
        glEnableVertexAttribArray(vertexColorLoc);

        // glDepthFunc ensure force is render on top of tethers, frames axis and ground
        glDepthFunc(GL_ALWAYS);
        glDrawArrays(GL_LINES, 0, 2);
        glDepthFunc(GL_LESS); // restore default

        m_vertexBufferForce.unbind();
        m_vertexArrayObjectForce.unbind();
    }

    shader.unbind();
};

// Update the transform matrix of the object
void FrameAxis::updateTransform(const double interpolationFactor) {
    // Get the transform of the rigid body
    const Transform transform = m_frame->transform();

    // Interpolate the transform between the previous one and the new one
    const Transform interpolatedTransform = Transform::interpolateTransforms(m_previousTransform,transform, interpolationFactor);
    m_previousTransform = transform;

    // Compute the transform used for rendering the box
    double matrix[16];
    interpolatedTransform.getOpenGLMatrix(matrix);
    const Matrix4 newMatrix = {{matrix[0], matrix[4], matrix[8], matrix[12]},
                              {matrix[1], matrix[5], matrix[9], matrix[13]},
                              {matrix[2], matrix[6], matrix[10], matrix[14]},
                              {matrix[3], matrix[7], matrix[11], matrix[15]}};

    // Apply the scaling matrix to have the correct box dimensions
    setTransformMatrix(newMatrix);
}

void FrameAxis::updateForceVector() const {
    auto force = m_frame->originNode().force();

    if (force.length() > 1e-6f) {
        force = force.normalize();
    }

    // Scale to length for visualization
    force = force * m_length;

    const FrameLine line(Vector3<float>::zeros(), cast<float>(force.toOpenglRef()), static_cast<uint32_t>(m_frame->getForceColor()));

    m_vertexBufferForce.bind();
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(FrameLine), &line);
    m_vertexBufferForce.unbind();
}

void FrameAxis::m_createVertexBufferAndArrayObject() {

    const auto x = FrameLine(Vector3<float>::zeros(), Vector3<float>(m_length, 0, 0).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::RED));
    const auto y = FrameLine(Vector3<float>::zeros(), Vector3<float>(0, m_length, 0).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::BLUE));
    const auto z = FrameLine(Vector3<float>::zeros(), Vector3<float>(0, 0, m_length).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::GREEN));

    const std::vector<FrameLine> vertices = {x,y,z};

    m_vertexBufferVertices.create();
    m_vertexBufferVertices.bind();
    const auto sizeVertices = static_cast<GLsizei>(vertices.size() * sizeof(FrameLine));
    m_vertexBufferVertices.copyDataIntoVBO(sizeVertices, &vertices[0], GL_STATIC_DRAW);
    m_vertexBufferVertices.unbind();

    m_vertexArrayObject.create();
    m_vertexArrayObject.bind();
    m_vertexBufferVertices.bind();
    m_vertexArrayObject.unbind();
}

void FrameAxis::m_createForceBufferAndArrayObject() {

    // Two vertices: origin and end of force vector
    const FrameLine line(Vector3<float>::zeros(), Vector3<float>(0.0f, 0.0f, 0.0f).toOpenglRef(), static_cast<uint32_t>(m_frame->getForceColor()));

    m_vertexBufferForce.create();
    m_vertexBufferForce.bind();
    m_vertexBufferForce.copyDataIntoVBO(sizeof(FrameLine), &line, GL_DYNAMIC_DRAW);
    m_vertexBufferForce.unbind();

    m_vertexArrayObjectForce.create();
    m_vertexArrayObjectForce.bind();
    m_vertexBufferForce.bind();
    m_vertexArrayObjectForce.unbind();
}
