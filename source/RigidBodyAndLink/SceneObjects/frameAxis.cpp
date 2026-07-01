/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "frameAxis.h"

FrameAxis::FrameAxis(const frameRenderData& renderData, const float length)
    : m_frameRenderData(renderData)
    , m_vertexBufferVertices(GL_ARRAY_BUFFER)
    , m_vertexBufferForce(GL_ARRAY_BUFFER)
    , m_length(length)
{
    m_previousTransform = grs::TransformD::identity();

    m_createVertexBufferAndArrayObject();
    m_createForceBufferAndArrayObject();
}

FrameAxis::~FrameAxis() {
    // Destroy the VBOs and VAO
    m_vertexBufferVertices.destroy();
    m_vertexBufferForce.destroy();
}


void FrameAxis::render(const Shader& shader, const grs::Matrix4d& worldToCameraMatrix, const bool drawFrames, const bool drawForces) const {

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
        glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, sizeof(grs::Vec3f) + sizeof(uint32_t), nullptr);
        glEnableVertexAttribArray(vertexPositionLoc);

        // Color attribute
        glVertexAttribIPointer(vertexColorLoc, 1, GL_UNSIGNED_INT,  sizeof(grs::Vec3f) + sizeof(uint32_t), reinterpret_cast<void *>(sizeof(grs::Vec3f)));
        glEnableVertexAttribArray(vertexColorLoc);

        glDrawArrays(GL_LINES, 0, 6); // 3 arrows = 6 vertices

        m_vertexBufferVertices.unbind();
        m_vertexArrayObject.unbind();
    }


    if (drawForces) {
        m_vertexArrayObjectForce.bind();
        m_vertexBufferForce.bind();

        // Position attribute
        glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, sizeof(grs::Vec3f) + sizeof(uint32_t), nullptr);
        glEnableVertexAttribArray(vertexPositionLoc);

        // Color attribute
        glVertexAttribIPointer(vertexColorLoc, 1, GL_UNSIGNED_INT, sizeof(grs::Vec3f) + sizeof(uint32_t), reinterpret_cast<void*>(sizeof(grs::Vec3f)));
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
    const grs::TransformD transform = m_frameRenderData.transform;

    // Interpolate the transform between the previous one and the new one
    const grs::TransformD interpolatedTransform = grs::TransformD::interpolate(m_previousTransform,transform, interpolationFactor);
    m_previousTransform = transform;

    // Apply the scaling matrix
    setTransformMatrix(interpolatedTransform.toMatrix4());
}

void FrameAxis::updateForceVector() const {
    auto force = m_frameRenderData.forceAtOrigin;

    if (force.norm() > 1e-6f) {
        force = grs::normalize(force);
    }

    // Scale to length for visualization
    force = force * static_cast<double>(m_length);

    const FrameLine line(grs::Vec3f::zeros(), grs::cast<float>(force.toOpenglRef()), static_cast<uint32_t>(m_frameRenderData.forceColor));

    m_vertexBufferForce.bind();
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(FrameLine), &line);
    m_vertexBufferForce.unbind();
}

void FrameAxis::setFrameRenderData(const frameRenderData& renderData) {
    m_frameRenderData = renderData;
}

void FrameAxis::m_createVertexBufferAndArrayObject() {

    const auto x = FrameLine(grs::Vec3f::zeros(), grs::Vec3f(m_length, 0, 0).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::RED));
    const auto y = FrameLine(grs::Vec3f::zeros(), grs::Vec3f(0, m_length, 0).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::BLUE));
    const auto z = FrameLine(grs::Vec3f::zeros(), grs::Vec3f(0, 0, m_length).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::GREEN));

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
    const FrameLine line(grs::Vec3f::zeros(), grs::Vec3f::zeros(), static_cast<uint32_t>(m_frameRenderData.forceColor));

    m_vertexBufferForce.create();
    m_vertexBufferForce.bind();
    m_vertexBufferForce.copyDataIntoVBO(sizeof(FrameLine), &line, GL_DYNAMIC_DRAW);
    m_vertexBufferForce.unbind();

    m_vertexArrayObjectForce.create();
    m_vertexArrayObjectForce.bind();
    m_vertexBufferForce.bind();
    m_vertexArrayObjectForce.unbind();
}
