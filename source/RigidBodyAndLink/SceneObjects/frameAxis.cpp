/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "frameAxis.h"

#include "Definitions/shaderLayout.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

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

    // Arrow heads are seen from every side
    const GLboolean cullFace = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_CULL_FACE);

    if (drawFrames) {
        m_vertexArrayObject.bind();
        glDrawArrays(GL_TRIANGLES, 0, 3 * kArrowVertexCount);
        m_vertexArrayObject.unbind();
    }

    if (drawForces && m_hasForce) {
        m_vertexArrayObjectForce.bind();

        // glDepthFunc ensure force is render on top of tethers, frames axis and ground
        glDepthFunc(GL_ALWAYS);
        glDrawArrays(GL_TRIANGLES, 0, kArrowVertexCount);
        glDepthFunc(GL_LESS); // restore default

        m_vertexArrayObjectForce.unbind();
    }

    if (cullFace) glEnable(GL_CULL_FACE);

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
    const auto force = m_frameRenderData.forceAtOrigin;

    // No meaningful direction: draw nothing
    m_hasForce = force.norm() > 1e-6;
    if (!m_hasForce) return;

    // Direction only, at the frame axis length
    const grs::Vec3d direction = grs::normalize(force) * static_cast<double>(m_length);

    std::vector<Vertex> vertices;
    vertices.reserve(kArrowVertexCount);
    m_appendArrow(vertices, grs::cast<float>(direction.toOpenglRef()), static_cast<uint32_t>(m_frameRenderData.forceColor), 0.18f * m_length, 0.055f * m_length, 0.016f * m_length);

    m_vertexBufferForce.bind();
    glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)), vertices.data());
    m_vertexBufferForce.unbind();
}

void FrameAxis::m_appendArrow(std::vector<Vertex>& vertices, const grs::Vec3f& tip, const uint32_t color, const float headLength, const float headRadius, const float shaftRadius) {
    const float length = tip.norm();
    const grs::Vec3f d = tip * (1.0f / length);
    const float head = std::min(headLength, 0.5f * length);
    const grs::Vec3f base = d * (length - head);

    // Basis around the arrow direction
    const grs::Vec3f reference = std::abs(d[0]) < 0.9f ? grs::Vec3f(1, 0, 0) : grs::Vec3f(0, 1, 0);
    const grs::Vec3f u = grs::normalize(grs::cross(d, reference));
    const grs::Vec3f v = grs::cross(d, u);

    // Shaft: thin prism from the origin to the base of the head
    for (int i = 0; i < kShaftSegments; ++i) {
        const float a0 = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i) / kShaftSegments;
        const float a1 = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i + 1) / kShaftSegments;
        const grs::Vec3f o0 = (u * std::cos(a0) + v * std::sin(a0)) * shaftRadius;
        const grs::Vec3f o1 = (u * std::cos(a1) + v * std::sin(a1)) * shaftRadius;

        vertices.push_back({.position = o0, .color = color});
        vertices.push_back({.position = o1, .color = color});
        vertices.push_back({.position = base + o1, .color = color});

        vertices.push_back({.position = o0, .color = color});
        vertices.push_back({.position = base + o1, .color = color});
        vertices.push_back({.position = base + o0, .color = color});
    }

    // Base cap slightly darker so the cone reads as a solid
    const auto darker = [](const uint32_t c) {
        const uint32_t r = ((c >> 16) & 0xFFu) * 6 / 10, g = ((c >> 8) & 0xFFu) * 6 / 10, b = (c & 0xFFu) * 6 / 10;
        return (r << 16) | (g << 8) | b;
    };
    const uint32_t capColor = darker(color);

    for (int i = 0; i < kConeSegments; ++i) {
        const float a0 = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i) / kConeSegments;
        const float a1 = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i + 1) / kConeSegments;
        const grs::Vec3f p0 = base + (u * std::cos(a0) + v * std::sin(a0)) * headRadius;
        const grs::Vec3f p1 = base + (u * std::cos(a1) + v * std::sin(a1)) * headRadius;

        vertices.push_back({.position = p0, .color = color});
        vertices.push_back({.position = p1, .color = color});
        vertices.push_back({.position = tip, .color = color});

        vertices.push_back({.position = base, .color = capColor});
        vertices.push_back({.position = p1, .color = capColor});
        vertices.push_back({.position = p0, .color = capColor});
    }
}

void FrameAxis::setFrameRenderData(const frameRenderData& renderData) {
    m_frameRenderData = renderData;
}

void FrameAxis::m_createVertexBufferAndArrayObject() {

    std::vector<Vertex> vertices;
    vertices.reserve(3 * kArrowVertexCount);
    const float headLength = 0.18f * m_length;
    const float headRadius = 0.055f * m_length;
    const float shaftRadius = 0.012f * m_length;
    m_appendArrow(vertices, grs::Vec3f(m_length, 0, 0).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::RED), headLength, headRadius, shaftRadius);
    m_appendArrow(vertices, grs::Vec3f(0, m_length, 0).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::BLUE), headLength, headRadius, shaftRadius);
    m_appendArrow(vertices, grs::Vec3f(0, 0, m_length).toOpenglRef(), static_cast<uint32_t>(Color::HexColor::GREEN), headLength, headRadius, shaftRadius);

    m_vertexArrayObject.create();
    m_vertexArrayObject.bind();

    // POSITION + COLOR
    m_vertexBufferVertices.create();
    m_vertexBufferVertices.bind();
    m_vertexBufferVertices.copyDataIntoVBO(static_cast<GLsizei>(vertices.size() * sizeof(Vertex)), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(VertexLayout::Position, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(VertexLayout::Position);

    glVertexAttribIPointer(VertexLayout::Color, 1, GL_UNSIGNED_INT, sizeof(Vertex), reinterpret_cast<void *>(offsetof(Vertex, color)));
    glEnableVertexAttribArray(VertexLayout::Color);

    m_vertexArrayObject.unbind();
}

void FrameAxis::m_createForceBufferAndArrayObject() {

    // Room for one arrow, filled by updateForceVector()
    const std::vector<Vertex> vertices(kArrowVertexCount, Vertex{.position = grs::Vec3f::zeros(), .color = static_cast<uint32_t>(m_frameRenderData.forceColor)});

    m_vertexArrayObjectForce.create();
    m_vertexArrayObjectForce.bind();

    // POSITION + COLOR
    m_vertexBufferForce.create();
    m_vertexBufferForce.bind();
    m_vertexBufferForce.copyDataIntoVBO(static_cast<GLsizei>(vertices.size() * sizeof(Vertex)), vertices.data(), GL_DYNAMIC_DRAW);

    glVertexAttribPointer(VertexLayout::Position, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(VertexLayout::Position);

    glVertexAttribIPointer(VertexLayout::Color, 1, GL_UNSIGNED_INT, sizeof(Vertex), reinterpret_cast<void *>(offsetof(Vertex, color)));
    glEnableVertexAttribArray(VertexLayout::Color);

    m_vertexArrayObjectForce.unbind();
}
