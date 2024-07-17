/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "texture2D.h"

Texture2D::Texture2D()
    : m_id(0)
    , m_unit(0)
    , m_width(0)
    , m_height(0)
{

}

Texture2D::Texture2D(uint32_t width, uint32_t height, uint32_t internalFormat, uint32_t format, uint32_t type)
    : Texture2D()
{
    create(width, height, internalFormat, format, type);
}

Texture2D::~Texture2D() {
    destroy();
}

void Texture2D::create(uint32_t width, uint32_t height, uint32_t internalFormat, uint32_t format, uint32_t type, void *data) {
    destroy();

    m_width = width;
    m_height = height;

    // Create the OpenGL texture
    glGenTextures(1, &m_id);
    assert(m_id != 0);
    glBindTexture(GL_TEXTURE_2D, m_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, m_width, m_height, 0, format, type, data);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture2D::create(uint32_t width, uint32_t height, uint32_t internalFormat, uint32_t format, uint32_t type,
                       uint32_t minFilter, uint32_t maxFilter, uint32_t wrapS, uint32_t wrapT, void* data) {
    destroy();

    m_width = width;
    m_height = height;

    // Create the OpenGL texture
    glGenTextures(1, &m_id);
    assert(m_id != 0);
    glBindTexture(GL_TEXTURE_2D, m_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapS);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, maxFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, m_width, m_height, 0, format, type, data);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture2D::destroy() {
    if (m_id != 0) {
        glDeleteTextures(1, &m_id);
        m_id = 0;
        m_unit = 0;
        m_width = 0;
        m_height = 0;
    }
}

void Texture2D::bind() const {
    assert(m_id != 0);
    glActiveTexture(GL_TEXTURE0 + m_unit);
    glBindTexture(GL_TEXTURE_2D, m_id);
}

void Texture2D::unbind() const {
    assert(m_id != 0);
    glActiveTexture(GL_TEXTURE0 + m_unit);
    glBindTexture(GL_TEXTURE_2D, 0);
}

uint32_t Texture2D::getID() const {
    return m_id;
}

uint32_t Texture2D::getUnit() const {
    return m_unit;
}

void Texture2D::setUnit(uint32_t unit) {
    m_unit = unit;
}

uint32_t Texture2D::getWidth() const {
    return m_width;
}

uint32_t Texture2D::getHeight() const {
    return m_height;
}
