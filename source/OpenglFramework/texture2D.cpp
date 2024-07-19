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

Texture2D::Texture2D(unsigned int width, unsigned int height, unsigned int internalFormat, unsigned int format, unsigned int type)
    : Texture2D()
{
    create(width, height, internalFormat, format, type);
}

Texture2D::~Texture2D() {
    destroy();
}

void Texture2D::create(unsigned int width, unsigned int height, unsigned int internalFormat, unsigned int format, unsigned int type, void *data) {
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

void Texture2D::create(unsigned int width, unsigned int height, unsigned int internalFormat, unsigned int format, unsigned int type,
                       unsigned int minFilter, unsigned int maxFilter, unsigned int wrapS, unsigned int wrapT, void* data) {
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

unsigned int Texture2D::getID() const {
    return m_id;
}

unsigned int Texture2D::getUnit() const {
    return m_unit;
}

void Texture2D::setUnit(unsigned int unit) {
    m_unit = unit;
}
