/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "frameBufferObject.h"

FrameBufferObject::FrameBufferObject() 
    : m_frameBufferId(0)
    , m_renderBufferId(0)
{

}

FrameBufferObject::~FrameBufferObject() {
    destroy();
}

bool FrameBufferObject::create(const GLsizei width, const GLsizei height, const bool needRenderBuffer) {
    // Destroy the current FBO
    destroy();

    // Generate a new FBO
    glGenFramebuffers(1, &m_frameBufferId);
    if (m_frameBufferId == 0) {
        std::cerr << "Failed to generate framebuffer" << std::endl;
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferId);

    // If we also need to create a render buffer
    if (needRenderBuffer) {

        // Generate the render buffer
        glGenRenderbuffers(1, &m_renderBufferId);
        if (m_renderBufferId == 0) {
            std::cerr << "Failed to generate renderbuffer" << std::endl;
            return false;
        }

        glBindRenderbuffer(GL_RENDERBUFFER, m_renderBufferId);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,m_renderBufferId);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
    }

    // Check the FBO status
    const GLenum statusFBO = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if(statusFBO != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "FBO creation failed: status = 0x" << std::hex << statusFBO << std::endl;
        return false;
    }

    return true;
}

void FrameBufferObject::attachTexture(const GLenum position, const GLuint textureId) const {
    assert(m_frameBufferId);

    // Bind the current FBO
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferId);

    // Bind the texture
    glFramebufferTexture2D(GL_FRAMEBUFFER, position, GL_TEXTURE_2D, textureId, 0);

    // Unbind the current FBO
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FrameBufferObject::bind() const {
    assert(m_frameBufferId != 0);
    glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferId);
}

void FrameBufferObject::unbind() const {
    assert(m_frameBufferId != 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FrameBufferObject::destroy() {
    // Delete the frame buffer object
    if (m_frameBufferId) {
        glDeleteFramebuffers(1, &m_frameBufferId);
        m_frameBufferId = 0;
    }

    // Delete the render buffer
    if (m_renderBufferId) {
        glDeleteRenderbuffers(1, &m_renderBufferId);
    }
}
