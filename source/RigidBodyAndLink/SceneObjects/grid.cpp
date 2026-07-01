/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "grid.h"


Grid::Grid(const float extent, const float spacing) {
    m_createGrid(extent, spacing);
    m_createBuffers();
}

Grid::~Grid() {
    m_vertexBuffer.destroy();
    m_vertexArrayObject.destroy();
}

void Grid::m_addLine(const grs::Vec3f& p1, const grs::Vec3f& p2, const float width, const uint32_t color) {
    grs::Vec3f dir = p2 - p1;
    dir = grs::normalize(dir);

    // Grid lies on XZ plane
    grs::Vec3f perp(-dir[2], 0.0f, dir[0]);
    perp *= width * 0.5f;

    const grs::Vec3f v0 = p1 - perp;
    const grs::Vec3f v1 = p1 + perp;
    const grs::Vec3f v2 = p2 + perp;
    const grs::Vec3f v3 = p2 - perp;

    // Triangle 1
    m_vertices.push_back({v0, color});
    m_vertices.push_back({v1, color});
    m_vertices.push_back({v2, color});

    // Triangle 2
    m_vertices.push_back({v0, color});
    m_vertices.push_back({v2, color});
    m_vertices.push_back({v3, color});
}

void Grid::m_createGrid(const float extent, const float spacing) {
    m_vertices.clear();

    constexpr float minorWidth = 0.20f;
    constexpr float majorWidth = 0.20f;
    constexpr float axisWidth  = 0.20f;

    constexpr uint32_t gray  = static_cast<uint32_t>(0xD8D8D8);
    constexpr uint32_t red   = static_cast<uint32_t>(0xC85A5A);
    constexpr uint32_t green = static_cast<uint32_t>(0x5FAF5F);

    for (float x = -extent; x <= extent + 1e-5f; x += spacing) {
        uint32_t color = gray;
        float width = minorWidth;

        if (std::abs(x) < 1e-4f) {
            color = green;
            width = axisWidth;
        } else if (std::fmod(std::abs(x), 50.0f) < 1e-4f) {
            width = majorWidth;
        }

        m_addLine(grs::Vec3f(x, 0.01f, -extent),grs::Vec3f(x, 0.01f,  extent), width,color);
    }

    for (float z = -extent; z <= extent + 1e-5f; z += spacing) {
        uint32_t color = gray;
        float width = minorWidth;

        if (std::abs(z) < 1e-4f) {
            color = red;
            width = axisWidth;
        } else if (std::fmod(std::abs(z), 50.0f) < 1e-4f) {
            width = majorWidth;
        }

        m_addLine(grs::Vec3f(-extent, 0.01f, z), grs::Vec3f( extent, 0.01f, z), width, color);
    }
}

void Grid::m_createBuffers() {
    m_vertexBuffer.create();
    m_vertexBuffer.bind();

    m_vertexBuffer.copyDataIntoVBO(static_cast<GLsizei>(m_vertices.size() * sizeof(Vertex)),m_vertices.data(),GL_STATIC_DRAW);

    m_vertexBuffer.unbind();

    m_vertexArrayObject.create();
    m_vertexArrayObject.bind();

    m_vertexBuffer.bind();

    // Position attribute (location must match your shader)
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE, sizeof(Vertex),nullptr);
    glEnableVertexAttribArray(0);

    // Color attribute (uint32 packed RGB)
    glVertexAttribIPointer(1,1,GL_UNSIGNED_INT,sizeof(Vertex),reinterpret_cast<void *>(offsetof(Vertex, color)));
    glEnableVertexAttribArray(1);

    m_vertexBuffer.unbind();
    m_vertexArrayObject.unbind();
}

void Grid::render(const Shader& shader, const grs::Matrix4d& worldToCameraMatrix, const grs::Matrix4d& projectionMatrix) const {
    shader.bind();

    shader.setMatrix4x4Uniform("localToWorldMatrix", m_transformMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);
    shader.setMatrix4x4Uniform("projectionMatrix", projectionMatrix);

    shader.setIntUniform("isGlobalVertexColorEnabled",0,false);

    const GLint posLoc = shader.getAttribLocation("vertexPosition");
    const GLint colLoc = shader.getAttribLocation("vertexColor");

    m_vertexArrayObject.bind();
    m_vertexBuffer.bind();

    glVertexAttribPointer(posLoc,3,GL_FLOAT,GL_FALSE,sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(posLoc);

    glVertexAttribIPointer(colLoc,1,GL_UNSIGNED_INT,sizeof(Vertex), reinterpret_cast<void *>(offsetof(Vertex, color)));
    glEnableVertexAttribArray(colLoc);

    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(m_vertices.size()));

    m_vertexBuffer.unbind();
    m_vertexArrayObject.unbind();

    shader.unbind();
}