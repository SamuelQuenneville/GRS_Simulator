/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "skybox.h"

Skybox::Skybox(const std::vector<std::string>& faces)
    : m_texture(faces)
{
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices),m_vertices,GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3,GL_FLOAT,GL_FALSE, 3 * sizeof(float), nullptr);

    glBindVertexArray(0);
}

Skybox::Skybox(const std::string& crossImagePath)
    : m_texture(crossImagePath)
{
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER,sizeof(m_vertices),m_vertices,GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

    glBindVertexArray(0);
}

Skybox::~Skybox() {
    glDeleteVertexArrays(1, &m_vao);
    glDeleteBuffers(1, &m_vbo);
}

void Skybox::render(const Shader& shader, const grs::Matrix4d& view, const grs::Matrix4d& projection) const {
    glDepthFunc(GL_LEQUAL);

    shader.bind();

    // Remove translation from view matrix
    grs::Matrix4d viewNoTranslation = view;
    viewNoTranslation(0, 3) = 0.0;
    viewNoTranslation(1, 3) = 0.0;
    viewNoTranslation(2, 3) = 0.0;

    shader.setMatrix4x4Uniform("view", viewNoTranslation);
    shader.setMatrix4x4Uniform("projection", projection);

    shader.setIntUniform("skybox", 0, false);

    m_texture.bind(0);

    glBindVertexArray(m_vao);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);

    shader.unbind();

    glDepthFunc(GL_LESS);
}